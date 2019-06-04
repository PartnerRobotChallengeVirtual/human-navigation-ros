#include <ros/ros.h>
#include <human_navigation/HumanNaviObjectInfo.h>
#include <human_navigation/HumanNaviDestination.h>
#include <human_navigation/HumanNaviTaskInfo.h>
#include <human_navigation/HumanNaviMsg.h>
#include <human_navigation/HumanNaviGuidanceMsg.h>
#include <human_navigation/HumanNaviAvatarStatus.h>
#include <human_navigation/HumanNaviObjectStatus.h>

class HumanNavigationSample
{
private:
	enum Step
	{
		Initialize,
		Ready,
		WaitTaskInfo,
		GuideForTakingObject,
		GuideForPlacement,
		WaitTaskFinished,
		TaskFinished
	};

	enum class SpeechState
	{
		None,
		WaitingState,
		Speaking,
		Speakable
	};

	// human navigation message from/to the moderator
	const std::string MSG_ARE_YOU_READY      = "Are_you_ready?";
	const std::string MSG_TASK_SUCCEEDED     = "Task_succeeded";
	const std::string MSG_TASK_FAILED        = "Task_failed";
	const std::string MSG_TASK_FINISHED      = "Task_finished";
	const std::string MSG_GO_TO_NEXT_SESSION = "Go_to_next_session";
	const std::string MSG_MISSION_COMPLETE   = "Mission_complete";
	const std::string MSG_REQUEST            = "Guidance_request";
	const std::string MSG_SPEECH_STATE       = "Speech_state";
	const std::string MSG_SPEECH_RESULT      = "Speech_result";

	const std::string MSG_I_AM_READY        = "I_am_ready";
	const std::string MSG_GET_AVATAR_STATUS = "Get_avatar_status";
	const std::string MSG_GET_OBJECT_STATUS = "Get_object_status";
	const std::string MSG_GET_SPEECH_STATE  = "Get_speech_state";

	// display type of guidance message panels for the avatar (test subject)
	const std::string DISPLAY_TYPE_ALL         = "All";
	const std::string DISPLAY_TYPE_ROBOT_ONLY  = "RobotOnly";
	const std::string DISPLAY_TYPE_AVATAR_ONLY = "AvatarOnly";
	const std::string DISPLAY_TYPE_NONE        = "None";

	int step;
	SpeechState speechState;

	bool isStarted;
	bool isFinished;

	bool isTaskInfoReceived;
	bool isRequestReceived;

	ros::Time timePrevSpeechStateConfirmed;

	bool isSentGetAvatarStatus;
	bool isSentGetObjectStatus;

	human_navigation::HumanNaviTaskInfo taskInfo;
	std::string guideMsg;

	human_navigation::HumanNaviAvatarStatus avatarStatus;
	human_navigation::HumanNaviObjectStatus objectStatus;

	void init()
	{
		step = Initialize;
		speechState = SpeechState::None;

		reset();
	}

	void reset()
	{
		isStarted             = false;
		isFinished            = false;
		isTaskInfoReceived    = false;
		isRequestReceived     = false;
		isSentGetAvatarStatus = false;
		isSentGetObjectStatus = false;
	}

	// send humanNaviMsg to the moderator (Unity)
	void sendMessage(ros::Publisher &publisher, const std::string &message)
	{
		human_navigation::HumanNaviMsg human_navi_msg;
		human_navi_msg.message = message;
		publisher.publish(human_navi_msg);

		ROS_INFO("Send message:%s", message.c_str());
	}

	void sendGuidanceMessage(ros::Publisher &publisher, const std::string &message, const std::string displayType)
	{
		human_navigation::HumanNaviGuidanceMsg guidanceMessage;
		guidanceMessage.message = message;
		guidanceMessage.display_type = displayType;
		guidanceMessage.source_language = ""; // Blank or ISO-639-1 language code, e.g. "en".
		guidanceMessage.target_language = ""; // Blank or ISO-639-1 language code, e.g. "ja".
		publisher.publish(guidanceMessage);

		speechState = SpeechState::Speaking;

		ROS_INFO("Send guide message: %s : %s", guidanceMessage.message.c_str(), guidanceMessage.display_type.c_str());
	}


	// receive humanNaviMsg from the moderator (Unity)
	void messageCallback(const human_navigation::HumanNaviMsg::ConstPtr& message)
	{
		ROS_INFO("Subscribe message: %s : %s", message->message.c_str(), message->detail.c_str());

		if(message->message==MSG_ARE_YOU_READY)
		{
			isStarted = true;
		}
		else if(message->message==MSG_REQUEST)
		{
			if(isTaskInfoReceived && !isFinished)
			{
				isRequestReceived = true;
			}
		}
		else if(message->message==MSG_TASK_SUCCEEDED)
		{
		}
		else if(message->message==MSG_TASK_FAILED)
		{
		}
		else if(message->message==MSG_TASK_FINISHED)
		{
			isFinished = true;
		}
		else if(message->message==MSG_GO_TO_NEXT_SESSION)
		{
			ROS_INFO("Go to next session");
			step = Initialize;
		}
		else if(message->message==MSG_MISSION_COMPLETE)
		{
			//exit(EXIT_SUCCESS);
		}
		else if(message->message==MSG_SPEECH_STATE)
		{
			if(message->detail=="Is_speaking")
			{
				speechState = SpeechState::Speaking;
			}
			else
			{
				speechState = SpeechState::Speakable;
			}
		}
		else if(message->message==MSG_SPEECH_RESULT)
		{
			ROS_INFO("Speech result: %s", message->detail.c_str());
		}
	}

	// receive taskInfo from the moderator (Unity)
	void taskInfoMessageCallback(const human_navigation::HumanNaviTaskInfo::ConstPtr& message)
	{
		taskInfo = *message;

		ROS_INFO_STREAM(
			"Subscribe task info message:" << std::endl <<
			"Environment ID: " << taskInfo.environment_id << std::endl <<
			"Target object: " << std::endl << taskInfo.target_object <<
			"Destination: " << std::endl << taskInfo.destination
		);

		int numOfNonTargetObjects = taskInfo.non_target_objects.size();
		std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
		std::cout << "Non-target objects:" << std::endl;
		for(int i=0; i<numOfNonTargetObjects; i++)
		{
			std::cout << taskInfo.non_target_objects[i] << std::endl;
		}

		int numOfFurniture = taskInfo.furniture.size();
		std::cout << "Number of furniture: " << numOfFurniture << std::endl;
		std::cout << "Furniture objects:" << std::endl;
		for(int i=0; i<numOfFurniture; i++)
		{
			std::cout << taskInfo.furniture[i] << std::endl;
		}

		isTaskInfoReceived = true;
	}

	void avatarStatusMessageCallback(const human_navigation::HumanNaviAvatarStatus::ConstPtr& message)
	{
		avatarStatus = *message;

		ROS_INFO_STREAM(
			"Subscribe avatar status message:" << std::endl <<
			"Head: " << std::endl << avatarStatus.head << 
			"LeftHand: " << std::endl << avatarStatus.left_hand << 
			"rightHand: " << std::endl << avatarStatus.right_hand <<
			"objctInLeftHand: " << avatarStatus.object_in_left_hand << std::endl <<
			"objectInRightHand: " << avatarStatus.object_in_right_hand << std::endl <<
			"isTargetObjectInLeftHand: " << std::boolalpha << (bool)avatarStatus.is_target_object_in_left_hand << std::endl <<
			"isTargetObjectInRightHand: " << std::boolalpha << (bool)avatarStatus.is_target_object_in_right_hand << std::endl
		);
		isSentGetAvatarStatus = false;
	}

	void objectStatusMessageCallback(const human_navigation::HumanNaviObjectStatus::ConstPtr& message)
	{
		objectStatus = *message;

		ROS_INFO_STREAM(
			"Subscribe object status message:" << std::endl <<
			"Target object: " << std::endl << taskInfo.target_object
		);

		int numOfNonTargetObjects = taskInfo.non_target_objects.size();
		std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
		std::cout << "Non-target objects:" << std::endl;
		for(int i=0; i<numOfNonTargetObjects; i++)
		{
			std::cout << taskInfo.non_target_objects[i] << std::endl;
		}

		isSentGetObjectStatus = false;
	}

	bool speakGuidanceMessage(ros::Publisher pubHumanNaviMsg, ros::Publisher pubGuidanceMsg, std::string message, int interval = 1)
	{
		if(speechState == SpeechState::Speakable)
		{
			sendGuidanceMessage(pubGuidanceMsg, message, DISPLAY_TYPE_ALL);
			speechState = SpeechState::None;
			return true;
		}
		else if(speechState == SpeechState::None || speechState == SpeechState::Speaking)
		{
			if(timePrevSpeechStateConfirmed.sec + interval < ros::Time::now().sec)
			{
				sendMessage(pubHumanNaviMsg, MSG_GET_SPEECH_STATE);
				timePrevSpeechStateConfirmed = ros::Time::now();
				speechState = SpeechState::WaitingState;
			}
		}

		return false;
	}

public:
	int run(int argc, char **argv)
	{
		ros::NodeHandle nodeHandle;

		ros::Rate loopRate(10);

		init();

		ROS_INFO("Human Navi sample start!");

		ros::Subscriber subHumanNaviMsg = nodeHandle.subscribe<human_navigation::HumanNaviMsg>("/human_navigation/message/to_robot", 100, &HumanNavigationSample::messageCallback, this);
		ros::Subscriber subTaskInfoMsg = nodeHandle.subscribe<human_navigation::HumanNaviTaskInfo>("/human_navigation/message/task_info", 1, &HumanNavigationSample::taskInfoMessageCallback, this);
		ros::Subscriber subAvatarStatusMsg = nodeHandle.subscribe<human_navigation::HumanNaviAvatarStatus>("/human_navigation/message/avatar_status", 1, &HumanNavigationSample::avatarStatusMessageCallback, this);
		ros::Subscriber subObjectStatusMsg = nodeHandle.subscribe<human_navigation::HumanNaviObjectStatus>("/human_navigation/message/object_status", 1, &HumanNavigationSample::objectStatusMessageCallback, this);
		ros::Publisher pubHumanNaviMsg = nodeHandle.advertise<human_navigation::HumanNaviMsg>("/human_navigation/message/to_moderator", 10);
		ros::Publisher pubGuidanceMsg  = nodeHandle.advertise<human_navigation::HumanNaviGuidanceMsg>("/human_navigation/message/guidance_message", 10);

		ros::Time time;

		while (ros::ok())
		{
			switch(step)
			{
				case Initialize:
				{
					reset();

					ROS_INFO("##### Initialized ######");

					step++;
					break;
				}
				case Ready:
				{
					if(isStarted)
					{
						step++;

						sendMessage(pubHumanNaviMsg, MSG_I_AM_READY);

						ROS_INFO("Task start");
					}
					break;
				}
				case WaitTaskInfo:
				{
					if(isTaskInfoReceived){ step++; }
					break;
				}
				case GuideForTakingObject:
				{
					if(isRequestReceived)
					{
						isRequestReceived = false;
					}

					std::string targetObjectName;
					if(taskInfo.target_object.name.find("empty_plastic_bottle") != std::string::npos)
					{
						targetObjectName = "an empty plastic bottle ";
					}
					else
					{
						targetObjectName = "a cup ";
					}

					std::string locationName;
					if(taskInfo.target_object.position.x > 0.0)
					{
						locationName = "on a table.";
					}
					else
					{
						locationName = "next to the kitchen sink.";
					}

					guideMsg = "Please take " + targetObjectName + locationName;

					if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
					{
						time = ros::Time::now();
						step++;
					}
					break;
				}
				case GuideForPlacement:
				{
					if(isRequestReceived)
					{
						if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
						{
							isRequestReceived = false;
						}
					}

					int WaitTime = 5;
					if(time.sec + WaitTime < ros::Time::now().sec)
					{
						std::string destinationName;
						if(taskInfo.destination.position.z < 1.0)
						{
							destinationName = "a trash can on the left.";
						}
						else
						{
							destinationName = "the second cabinet from the right.";
						}
						guideMsg = "Put it in " + destinationName;

						if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
						{
							time = ros::Time::now();
							step++;
						}
					}

					break;
				}
				case WaitTaskFinished:
				{
					if(isFinished)
					{
						ROS_INFO("Task finished");
						step++;
						break;
					}

					if(isRequestReceived)
					{
						bool isSpeaked;
						if(ros::Time::now().sec % 2 > 0)
						{
							isSpeaked = speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg);
						}
						else
						{
							isSpeaked = speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, "You can find the wall cabinet above the kitchen sink.");
						}

						if(isSpeaked)
						{
							isRequestReceived = false;
						}
					}

					int WaitTime = 5;
					if(time.sec + WaitTime < ros::Time::now().sec)
					{
						if(!isSentGetAvatarStatus && !isSentGetObjectStatus)
						{
							sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
							sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
							isSentGetAvatarStatus = true;
							isSentGetObjectStatus = true;
							time = ros::Time::now();
						}
					}

					break;
				}
				case TaskFinished:
				{
					// Wait MSG_GO_TO_NEXT_SESSION or MSG_MISSION_COMPLETE
					break;
				}
			}

			ros::spinOnce();

			loopRate.sleep();
		}

		return 0;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_navi_sample");

	HumanNavigationSample humanNaviSample;

	humanNaviSample.run(argc, argv);
};


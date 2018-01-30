#include <ros/ros.h>
#include <std_msgs/String.h>
#include <human_navigation/HumanNaviObjectInfo.h>
#include <human_navigation/HumanNaviTaskInfo.h>
#include <human_navigation/HumanNaviMsg.h>
#include <human_navigation/HumanNaviAvatarPose.h>

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

	enum SpeechState
	{
		None,
		WaitingState,
		Speaking,
		Speakable
	};

	const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
	const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
	const std::string MSG_TASK_FAILED      = "Task_failed";
	const std::string MSG_TASK_FINISHED    = "Task_finished";
	const std::string MSG_GO_TO_NEXT_TRIAL = "Go_to_next_trial";
	const std::string MSG_MISSION_COMPLETE = "Mission_complete";
	const std::string MSG_REQUEST          = "Guidance_request";
	const std::string MSG_SPEECH_STATE     = "Speech_state";

	const std::string MSG_I_AM_READY           = "I_am_ready";
	const std::string MSG_GET_AVATAR_POSE      = "Get_avatar_pose";
	const std::string MSG_GET_OBJECT_POSITIONS = "Get_object_positions";
	const std::string MSG_CONFIRM_SPEECH_STATE = "Confirm_speech_state";

	int step;
	int speechState;

	bool isStarted;
	bool isFinished;

	bool isTaskInfoReceived;
	bool isRequestReceived;

	ros::Time timePrevSpeechStateConfirmed;

	bool isSentGetAvatarPose;

	human_navigation::HumanNaviTaskInfo taskInfo;
	std::string guideMsg;

	human_navigation::HumanNaviAvatarPose avatarPose;

	void init()
	{
		step = Initialize;
		speechState = None;

		reset();
	}

	void reset()
	{
		isStarted           = false;
		isFinished          = false;
		isTaskInfoReceived  = false;
		isRequestReceived   = false;
		isSentGetAvatarPose = false;
	}

	// send humanNaviMsg to the moderator (Unity)
	void sendMessage(ros::Publisher &publisher, const std::string &message)
	{
		human_navigation::HumanNaviMsg human_navi_msg;
		human_navi_msg.message = message;
		publisher.publish(human_navi_msg);

		ROS_INFO("Send message:%s", message.c_str());
	}

	// send guide message to the virtual robot (Unity)
	void sendStringMessage(ros::Publisher &publisher, const std::string &message)
	{
		std_msgs::String stringMsg;
		stringMsg.data = message;
		publisher.publish(stringMsg);

		ROS_INFO("Send guide message: %s", message.c_str());
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
		else if(message->message==MSG_GO_TO_NEXT_TRIAL)
		{
			ROS_INFO("Go to next trial");
			step = Initialize;
		}
		else if(message->message==MSG_MISSION_COMPLETE)
		{
			exit(EXIT_SUCCESS);
		}
		else if(message->message==MSG_SPEECH_STATE)
		{
			if(message->detail=="Is_speaking")
			{
				speechState = Speaking;
			}
			else
			{
				speechState = Speakable;
			}
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

		/*int objectNum = taskInfo.objects_info.size();
		std::cout << "Number of other objects: " << objectNum << std::endl;
		std::cout << "Other objects:" << std::endl;
		for(int i=0; i<objectNum; i++)
		{
			std::cout << taskInfo.objects_info[i] << std::endl;
		}*/

		isTaskInfoReceived = true;
	}

	void avatarPoseMessageCallback(const human_navigation::HumanNaviAvatarPose::ConstPtr& message)
	{
		avatarPose = *message;

		ROS_INFO_STREAM(
			"Head: " << std::endl << avatarPose.head << 
			"LeftHand: " << std::endl << avatarPose.left_hand << 
			"rightHand: " << std::endl << avatarPose.right_hand
		);
		isSentGetAvatarPose = false;
	}

	bool speakGuidanceMessage(ros::Publisher pubHumanNaviMsg, ros::Publisher pubStringMsg, std::string message, int interval = 1)
	{
		if(speechState == Speakable)
		{
			sendStringMessage(pubStringMsg, message);
			speechState = None;
			return true;
		}
		else if(speechState == None || speechState == Speaking)
		{
			if(timePrevSpeechStateConfirmed.sec + interval < ros::Time::now().sec)
			{
				sendMessage(pubHumanNaviMsg, MSG_CONFIRM_SPEECH_STATE);
				timePrevSpeechStateConfirmed = ros::Time::now();
				speechState = WaitingState;
			}
		}

		return false;
	}

public:
	int run(int argc, char **argv)
	{
		ros::init(argc, argv, "human_navi_sample");

		ros::NodeHandle nodeHandle;

		ros::Rate loopRate(10);

		init();

		ROS_INFO("Human Navi sample start!");

		ros::Subscriber subHumanNaviMsg = nodeHandle.subscribe<human_navigation::HumanNaviMsg>("/human_navigation/message/to_robot", 100, &HumanNavigationSample::messageCallback, this);
		ros::Subscriber subTaskInfoMsg = nodeHandle.subscribe<human_navigation::HumanNaviTaskInfo>("/human_navigation/message/task_info", 1, &HumanNavigationSample::taskInfoMessageCallback, this);
		ros::Subscriber subAvatarPoseMsg = nodeHandle.subscribe<human_navigation::HumanNaviAvatarPose>("/human_navigation/message/avatar_pose", 1, &HumanNavigationSample::avatarPoseMessageCallback, this);
		ros::Publisher pubHumanNaviMsg = nodeHandle.advertise<human_navigation::HumanNaviMsg>("/human_navigation/message/to_moderator", 10);
		ros::Publisher pubStringMsg  = nodeHandle.advertise<std_msgs::String>("/human_navigation/message/guidance_message", 10);

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
					if(taskInfo.target_object.name.find("petbottle_500ml_empty_01") != std::string::npos)
					{
						targetObjectName = "an empty plastic bottle ";
					}
					else
					{
						targetObjectName = "a cup ";
					}

					std::string locationName;
					if(taskInfo.target_object.position.z > 0.0)
					{
						locationName = "on a table.";
					}
					else
					{
						locationName = "next to the kitchen sink.";
					}

					guideMsg = "Please take " + targetObjectName + locationName;

					if(speakGuidanceMessage(pubHumanNaviMsg, pubStringMsg, guideMsg))
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
						if(speakGuidanceMessage(pubHumanNaviMsg, pubStringMsg, guideMsg))
						{
							isRequestReceived = false;
						}
					}

					int WaitTime = 5;
					if(time.sec + WaitTime < ros::Time::now().sec)
					{
						std::string destinationName;
						if(taskInfo.destination.y < 1.0)
						{
							destinationName = "a trash box on the left.";
						}
						else
						{
							destinationName = "the second cabinet from the right.";
						}
						guideMsg = "Put it in " + destinationName;

						if(speakGuidanceMessage(pubHumanNaviMsg, pubStringMsg, guideMsg))
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
							isSpeaked = speakGuidanceMessage(pubHumanNaviMsg, pubStringMsg, guideMsg);
						}
						else
						{
							isSpeaked = speakGuidanceMessage(pubHumanNaviMsg, pubStringMsg, "You can find the cabinet above the kitchen sink.");
						}

						if(isSpeaked)
						{
							isRequestReceived = false;
						}
					}

					int WaitTime = 10;
					if(time.sec + WaitTime < ros::Time::now().sec && !isSentGetAvatarPose)
					{
						sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_POSE);

						time = ros::Time::now();
						isSentGetAvatarPose = true;
					}

					break;
				}
				case TaskFinished:
				{
					// Wait MSG_GO_TO_NEXT_TRIAL or MSG_MISSION_COMPLETE
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
	HumanNavigationSample humanNaviSample;

	humanNaviSample.run(argc, argv);
};


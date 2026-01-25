#include "ros_tf_broadcaster.h"
#include <thread>
#include <chrono>

namespace rosbridge2cpp {
	void ROSTFBroadcaster::SendTransform(json &geometry_msgs_transformstamped_msg)
	{
		assert(geometry_msgs_transformstamped_msg.IsObject());

		rapidjson::Document transform_array;
		transform_array.SetArray();
		transform_array.PushBack(geometry_msgs_transformstamped_msg, transform_array.GetAllocator());

		SendTransforms(transform_array);
	}

	void ROSTFBroadcaster::SendTransforms(json &geometry_msgs_transformstamped_array_msg)
	{
		assert(geometry_msgs_transformstamped_array_msg.IsArray());

		rapidjson::Document tf_message;
		tf_message.SetObject();

		tf_message.AddMember("transforms", geometry_msgs_transformstamped_array_msg, tf_message.GetAllocator());

		tf_topic_.Publish(tf_message);
	}

	void ROSTFBroadcaster::SendTransform(bson_t &bson)
	{
		tf_topic_.Publish(&bson);
	}

	void ROSTFBroadcaster::SendStaticTransforms(json &geometry_msgs_transformstamped_array_msg)
	{
		assert(geometry_msgs_transformstamped_array_msg.IsArray());

		rapidjson::Document tf_message;
		tf_message.SetObject();

		tf_message.AddMember("transforms", geometry_msgs_transformstamped_array_msg, tf_message.GetAllocator());

		tf_static_topic_.Publish(tf_message);
	}

	bool ROSTFBroadcaster::AdvertiseStaticTopic()
	{
		// Always ensure /tf_static is advertised before publishing
		// Unadvertise first to force re-advertisement (handles stale state after reconnection)
		tf_static_topic_.Unadvertise();
		bool success = tf_static_topic_.Advertise();
		// Small delay to allow rosbridge to process the advertise message
		if (success) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		return success;
	}
}

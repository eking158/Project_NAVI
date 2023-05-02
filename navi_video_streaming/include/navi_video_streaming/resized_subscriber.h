#include <image_transport/simple_subscriber_plugin.h>
#include <navi_video_streaming/ResizedImage.h>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<navi_video_streaming::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void internalCallback(const typename navi_video_streaming::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
};

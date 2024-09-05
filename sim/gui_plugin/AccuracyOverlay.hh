#ifndef ACCURACY_OVERLAY_HH
#define ACCURACY_OVERLAY_HH

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <QString>

// Gazebo GUI plugin that shows the latest gz.msgs.StringMsg of a topic as a
// text overlay. Config: <topic>/positioning/overlay</topic>
class AccuracyOverlay final : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY (QString text READ Text NOTIFY TextChanged)

public:
  AccuracyOverlay ();
  ~AccuracyOverlay () override;

  void LoadConfig (const tinyxml2::XMLElement *_pluginElem) override;

  Q_INVOKABLE QString Text () const;

signals:
  void TextChanged ();

private:
  void OnMessage (const gz::msgs::StringMsg &_msg);

  gz::transport::Node node{};
  QString text{ "waiting for positioning_sim..." };
};

#endif // ACCURACY_OVERLAY_HH

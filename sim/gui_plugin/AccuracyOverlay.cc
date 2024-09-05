#include "AccuracyOverlay.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <QMetaObject>
#include <string>

AccuracyOverlay::AccuracyOverlay () = default;

AccuracyOverlay::~AccuracyOverlay () = default;

void
AccuracyOverlay::LoadConfig (const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty ())
    {
      this->title = "Prediction accuracy";
    }

  std::string topic = "/positioning/overlay";
  if (_pluginElem)
    {
      if (const auto *elem = _pluginElem->FirstChildElement ("topic"))
        {
          if (elem->GetText ())
            {
              topic = elem->GetText ();
            }
        }
    }

  if (!this->node.Subscribe (topic, &AccuracyOverlay::OnMessage, this))
    {
      gzerr << "AccuracyOverlay: cannot subscribe to [" << topic << "]\n";
    }
}

QString
AccuracyOverlay::Text () const
{
  return this->text;
}

void
AccuracyOverlay::OnMessage (const gz::msgs::StringMsg &_msg)
{
  // Called on a transport thread, hand the text over to the Qt thread.
  const QString newText = QString::fromStdString (_msg.data ());
  QMetaObject::invokeMethod (
      this,
      [this, newText] ()
        {
          this->text = newText;
          emit this->TextChanged ();
        },
      Qt::QueuedConnection);
}

GZ_ADD_PLUGIN (AccuracyOverlay, gz::gui::Plugin)

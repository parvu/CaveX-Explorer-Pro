#include "ManualControl.hh"

#include <gz/plugin/Register.hh>
#include <gz/msgs/stringmsg.pb.h>

namespace sic_slam_gui
{

ManualControl::ManualControl() : gz::gui::Plugin()
{
  this->cmdPub = this->node.Advertise<gz::msgs::StringMsg>("/sic_slam/manual_cmd");
}

ManualControl::~ManualControl() = default;

void ManualControl::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Manual control";
  }
  if (_pluginElem) {
    if (auto elem = _pluginElem->FirstChildElement("mode")) {
      if (elem->GetText()) {
        this->mode = elem->GetText();
      }
    }
  }
}

QString ManualControl::Mode() const
{
  return this->mode;
}

void ManualControl::SendCommand(const QString & _cmd)
{
  gz::msgs::StringMsg msg;
  msg.set_data(_cmd.toStdString());
  this->cmdPub.Publish(msg);
}

}  // namespace sic_slam_gui

GZ_ADD_PLUGIN(sic_slam_gui::ManualControl, gz::gui::Plugin)

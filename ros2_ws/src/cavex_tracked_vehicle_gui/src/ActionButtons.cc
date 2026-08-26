#include "ActionButtons.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/stringmsg.pb.h>

namespace cavex_tracked_vehicle_gui
{

ActionButtons::ActionButtons() : gz::gui::Plugin()
{
}

ActionButtons::~ActionButtons() = default;

void ActionButtons::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Actions";
  }

  std::string topic;
  if (_pluginElem) {
    if (auto elem = _pluginElem->FirstChildElement("title")) {
      if (elem->GetText()) {
        this->title = elem->GetText();
      }
    }
    if (auto elem = _pluginElem->FirstChildElement("label")) {
      if (elem->GetText()) {
        this->label = elem->GetText();
      }
    }
    if (auto elem = _pluginElem->FirstChildElement("button1_label")) {
      if (elem->GetText()) {
        this->button1Label = elem->GetText();
      }
    }
    if (auto elem = _pluginElem->FirstChildElement("button1_cmd")) {
      if (elem->GetText()) {
        this->button1Cmd = elem->GetText();
      }
    }
    if (auto elem = _pluginElem->FirstChildElement("button2_label")) {
      if (elem->GetText()) {
        this->button2Label = elem->GetText();
      }
    }
    if (auto elem = _pluginElem->FirstChildElement("button2_cmd")) {
      if (elem->GetText()) {
        this->button2Cmd = elem->GetText();
      }
    }
    if (auto elem = _pluginElem->FirstChildElement("topic")) {
      if (elem->GetText()) {
        topic = elem->GetText();
      }
    }
  }

  if (topic.empty()) {
    gzerr << "ActionButtons [" << this->title << "] has no <topic> configured, "
             "buttons will do nothing\n";
  } else {
    this->controlPub = this->node.Advertise<gz::msgs::StringMsg>(topic);
  }

  this->LabelChanged();
  this->Button1LabelChanged();
  this->Button2LabelChanged();
}

QString ActionButtons::Label() const
{
  return this->label;
}

QString ActionButtons::Button1Label() const
{
  return this->button1Label;
}

QString ActionButtons::Button2Label() const
{
  return this->button2Label;
}

void ActionButtons::SendButton1()
{
  this->SendCommand(this->button1Cmd.toStdString());
}

void ActionButtons::SendButton2()
{
  this->SendCommand(this->button2Cmd.toStdString());
}

void ActionButtons::SendCommand(const std::string & _cmd)
{
  if (_cmd.empty()) {
    return;
  }
  gz::msgs::StringMsg msg;
  msg.set_data(_cmd);
  this->controlPub.Publish(msg);
}

}  // namespace cavex_tracked_vehicle_gui

GZ_ADD_PLUGIN(cavex_tracked_vehicle_gui::ActionButtons, gz::gui::Plugin)

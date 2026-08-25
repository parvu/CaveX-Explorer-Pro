#include "InfoLabel.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <QQmlContext>
#include <QMetaObject>

namespace sic_slam_gui
{

InfoLabel::InfoLabel() : gz::gui::Plugin()
{
}

InfoLabel::~InfoLabel() = default;

void InfoLabel::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Info";
  }

  std::string topic = "/sic_slam/info";
  if (_pluginElem) {
    if (auto elem = _pluginElem->FirstChildElement("topic")) {
      if (elem->GetText()) {
        topic = elem->GetText();
      }
    }
    if (auto elem = _pluginElem->FirstChildElement("title")) {
      if (elem->GetText()) {
        this->title = elem->GetText();
      }
    }
  }

  if (!this->node.Subscribe(topic, &InfoLabel::OnMessage, this)) {
    gzerr << "InfoLabel failed to subscribe to [" << topic << "]\n";
  }
}

QString InfoLabel::Text() const
{
  return this->text;
}

void InfoLabel::SetText(const QString & _text)
{
  this->text = _text;
  this->TextChanged();
}

void InfoLabel::OnMessage(const gz::msgs::StringMsg & _msg)
{
  // gz-transport invokes this on its own thread; SetText/TextChanged touch
  // a QObject that lives on the Qt GUI thread, so the update is marshaled
  // over with a queued connection instead of called directly.
  QString newText = QString::fromStdString(_msg.data());
  QMetaObject::invokeMethod(this, [this, newText]() {
    this->SetText(newText);
  }, Qt::QueuedConnection);
}

}  // namespace sic_slam_gui

GZ_ADD_PLUGIN(sic_slam_gui::InfoLabel, gz::gui::Plugin)

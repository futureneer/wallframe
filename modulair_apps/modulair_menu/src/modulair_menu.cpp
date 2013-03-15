#include <modulair_menu/modulair_menu.h>

namespace modulair{

ModulairMenu::ModulairMenu(ros::NodeHandle n)
{
	node_ = n;
	window_ = new ModulairMainWindow(NULL,"Menu",n);
}

ModulairMenu::~ModulairMenu(){}

void ModulairMenu::build()
{
	window_->build();
}


} // namespace

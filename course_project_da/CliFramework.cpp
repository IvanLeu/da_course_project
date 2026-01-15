#include "CliFramework.h"

cli::OptionContainer::OptionContainer(const char* description, const char* name)
	:
	app(description, name)
{}

void cli::OptionContainer::Finalize(int argc, char** argv) {
	app.parse(argc, argv);
}

int cli::OptionContainer::Exit(const CLI::ParseError& e) {
	return app.exit(e);
}

cli::Flag::Flag(OptionContainer* pParent, std::string name, std::string description)
{
	pOption = pParent->app.add_flag(std::move(name), data, std::move(description));
}

bool cli::Flag::operator*() const {
	return operator bool();
}

cli::Flag::operator bool() const {
	return static_cast<bool>(*pOption);
}

bool cli::Flag::operator!() const {
	return !(static_cast<bool>(*this));
}

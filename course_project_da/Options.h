#pragma once

#include "CliFramework.h"
#include <filesystem>

namespace options 
{
using namespace cli;

struct Options : public OptionBase<Options> {
	Option<std::filesystem::path> nodes{ this, "--nodes", {}, "Path to nodes file" };
	Option<std::filesystem::path> edges{ this, "--edges", {}, "Path to edges file" };
	Option<std::filesystem::path> graph{ this, "--graph", {}, "Path to graph file" };
	Option<std::filesystem::path> input{ this, "--input", {}, "Path to input file" };
	Option<std::filesystem::path> output{ this, "--output", {}, "Path to output file" };
	Flag preprocess{ this, "--preprocess", "Run in preprocessor mode" };
	Flag search{ this, "--search", "Run in search mode" };
	Flag full_output{ this, "--full-output", "Use verbouse output strategy" };
	static constexpr const char* name = "App";
	static constexpr const char* description = "App to find most optimal path from A to B";
};
}
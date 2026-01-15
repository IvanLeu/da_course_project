#include "Options.h"
#include "ErrorCodes.h"

#include <print>

#include "Preprocessor.h"
#include "Searcher.h"

int main(int argc, char** argv)
{
	using namespace options;

	if (auto e = Options::Init(argc, argv)) {
		return *e;
	}

	auto& opts = Options::Get();

	if (opts.preprocess)
	{
		if (!opts.nodes || !opts.edges)
		{
			std::println("--nodes and --edges must be specified in preprocess mode");
			return -1;
		}

		auto nodes = *opts.nodes;
		auto edges = *opts.edges;
		auto output = opts.output ? *opts.output : std::filesystem::current_path().append("assets\\output.graph");

		if (auto rc = DoPreprocess(std::move(nodes), std::move(edges), std::move(output)); rc == S_OK)
		{
			std::println("Successfully written into: {}", output.string());
		}
		else
		{
			std::println("Preprocessing failed. Error code: {}", rc);
		}
	}
	else if(opts.search)
	{
		if (!opts.graph || !opts.input)
		{
			std::println("--graph and --input must be specified in search mode");
			return -1;
		}

		auto graph = *opts.graph;
		auto input = *opts.input;
		auto output = opts.output ? *opts.output : std::filesystem::current_path().append("assets\\output.txt");
		auto full_output = opts.output ? *opts.full_output : false;

		if (auto rc = DoSearch(std::move(graph), std::move(input), std::move(output), full_output); rc == S_OK)
		{
			std::println("Successfully written into: {}", output.string());
		}
		else 
		{
			std::println("Searching failed. Error code: {}", rc);
		}
	}
	else
	{
		std::println("Use --help for usage");
	}

	return 0;
}
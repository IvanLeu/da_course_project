#pragma once

#include "ErrorCodes.h"
#include "Structs.h"
#include "Utils.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <sstream>
#include <string>
#include <vector>

inline int DoPreprocess(std::filesystem::path nodesPath,
	std::filesystem::path edgesPath,
	const std::filesystem::path& outputPath)
{
	std::vector<SavedNode> nodes;
	std::ifstream nodesFile{ nodesPath };

	if (!nodesFile)
		return ERR_NOT_FOUND;

	// Reserve close to desired size
	nodes.reserve(std::filesystem::file_size(nodesPath) / 30);

	std::copy(std::istream_iterator<SavedNode>(nodesFile), std::istream_iterator<SavedNode>{}, std::back_inserter(nodes));
	nodesFile.close();

	std::ifstream edgesFile{ edgesPath };
	std::ofstream outputFile{ outputPath, std::ios::binary };

	if (!edgesFile)
		return ERR_NOT_FOUND;

	const size_t nodeCount = nodes.size();
	outputFile.write(reinterpret_cast<const char*>(&nodeCount), sizeof(size_t));
	outputFile.write(reinterpret_cast<const char*>(nodes.data()), sizeof(SavedNode) * nodeCount);

	std::uint32_t k;
	while (edgesFile >> k)
	{
		if (k < 2)
		{
			std::uint32_t trash;
			for (auto i = 0u; i < k; ++i) edgesFile >> trash;
			continue;
		}

		std::uint32_t prev;
		edgesFile >> prev;

		std::int64_t prevIdx = ConvertToArrayIndex(prev, nodes);
		
		if (prevIdx == static_cast<std::uint32_t>(-1))
		{
			return ERR_CORRUPTED_DATA;
		}

		for (auto i = 1u; i < k; ++i)
		{
			std::uint32_t cur;
			edgesFile >> cur;

			std::int64_t curIdx = ConvertToArrayIndex(cur, nodes);

			if (curIdx == static_cast<std::uint32_t>(-1))
			{
				return ERR_CORRUPTED_DATA;
			}

			float distance = EuclideanDistance(nodes[prevIdx], nodes[curIdx]);

			SavedEdge e1{ prevIdx, curIdx, distance };
			outputFile.write(reinterpret_cast<const char*>(&e1), sizeof(SavedEdge));

			prev = cur;
			prevIdx = curIdx;
		}
	}

	edgesFile.close();
	outputFile.close();

	return S_OK;
}
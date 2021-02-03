#include <iostream>
#include <fstream>
#include <cassert>
#include <vector>
#include "algorithms/extract/region.hpp"

namespace odgi {

    namespace algorithms {

        void parse_region(const std::string &target, std::string &name, int64_t &start, int64_t &end) {
            start = -1;
            end = -1;
            size_t foundFirstColon = target.find(":");
            // we only have a single string, use the whole sequence as the target
            if (foundFirstColon == std::string::npos) {
                name = target;
            } else {
                name = target.substr(0, foundFirstColon);
                size_t foundRangeDash = target.find("-", foundFirstColon);
                if (foundRangeDash == std::string::npos) {
                    start = atoi(target.substr(foundFirstColon + 1).c_str());
                    end = start;
                } else {
                    start = atoi(target.substr(foundFirstColon + 1, foundRangeDash - foundRangeDash - 1).c_str());
                    end = atoi(target.substr(foundRangeDash + 1).c_str());
                }
            }
        }

        void parse_bed_regions(const std::string &bed_path,
                               std::vector<Region> &out_regions,
                               std::vector<std::string> *out_names) {
            out_regions.clear();
            std::ifstream bedstream(bed_path);
            if (!bedstream) {
                std::cerr << "Unable to open bed file: " << bed_path << std::endl;
                return;
            }
            std::string row;
            std::string sbuf;
            std::string ebuf;
            std::string nbuf;
            for (int line = 1; getline(bedstream, row); ++line) {
                Region region;
                if (row.size() < 2 || row[0] == '#') {
                    continue;
                }
                std::istringstream ss(row);
                if (!getline(ss, region.seq, '\t') ||
                    !getline(ss, sbuf, '\t') ||
                    !getline(ss, ebuf, '\t') ||
                    (out_names != nullptr && !getline(ss, nbuf, '\t'))) {
                    std::cerr << "Error parsing bed line " << line << ": " << row << std::endl;
                } else {
                    region.start = std::stoi(sbuf);
                    region.end = std::stoi(ebuf);
                    assert(region.end > region.start);

                    // convert from BED-style to 0-based inclusive coordinates
                    region.end -= 1;

                    out_regions.push_back(region);

                    if (out_names != nullptr) {
                        out_names->push_back(nbuf);
                    }
                }
            }
        }

    }
}
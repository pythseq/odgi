#include "subcommand.hpp"
#include "odgi.hpp"
#include "args.hxx"
#include "algorithms/extract/region.hpp"

namespace odgi {

    using namespace odgi::subcommand;

    int main_extract(int argc, char **argv) {

        // trick argumentparser to do the right thing with the subcommand
        for (uint64_t i = 1; i < argc - 1; ++i) {
            argv[i] = argv[i + 1];
        }
        std::string prog_name = "odgi extract";
        argv[0] = (char *) prog_name.c_str();
        --argc;

        args::ArgumentParser parser("extract pieces of the graph.");

        args::HelpFlag help(parser, "help", "display this help summary", {'h', "help"});
        args::ValueFlag<std::string> dg_in_file(parser, "FILE", "load the graph from this file", {'i', "idx"});
        args::ValueFlag<std::string> dg_out_file(parser, "FILE",
                                                 "store the graph with the generated paths in this file", {'o', "out"});

        args::ValueFlag<std::string> _path_target(parser, "STRING",
                                                  "find the node(s) in the specified path range(s) TARGET=path[:pos1[-pos2]]",
                                                  {'p', "path-target"});
        //args::ValueFlag<std::string> _path_target_bed(parser, "FILE", "read the path targets from the given BED FILE", {'P', "path-target-bed"});

        //args::ValueFlag<uint64_t> nthreads(parser, "N", "number of threads to use for the parallel sorter", {'t', "threads"});
        //args::Flag debug(parser, "debug", "print information about the components and the progress to stderr",{'d', "debug"});

        try {
            parser.ParseCLI(argc, argv);
        } catch (args::Help) {
            std::cout << parser;
            return 0;
        } catch (args::ParseError e) {
            std::cerr << e.what() << std::endl;
            std::cerr << parser;
            return 1;
        }
        if (argc == 1) {
            std::cout << parser;
            return 1;
        }

        if (!dg_in_file) {
            std::cerr
                    << "[odgi extract] error: please specify an input file from where to load the graph via -i=[FILE], --idx=[FILE]."
                    << std::endl;
            return 1;
        }

        if (!dg_out_file) {
            std::cerr
                    << "[odgi extract] error: please specify an output file to where to store the graph via -o=[FILE], --out=[FILE]."
                    << std::endl;
            return 1;
        }

        std::vector<string> targets_str;
        std::vector<Region> targets;


        graph_t graph;
        assert(argc > 0);
        std::string infile = args::get(dg_in_file);
        if (infile.size()) {
            if (infile == "-") {
                graph.deserialize(std::cin);
            } else {
                ifstream f(infile.c_str());
                graph.deserialize(f);
                f.close();
            }
        }


        //uint64_t num_threads = args::get(nthreads) ? args::get(nthreads) : 1;


        std::string outfile = args::get(dg_out_file);
        if (outfile.size()) {
            if (outfile == "-") {
                graph.serialize(std::cout);
            } else {
                ofstream f(outfile.c_str());
                graph.serialize(f);
                f.close();
            }
        }
        return 0;
    }

    static Subcommand odgi_extract("extract", "extract pieces of the graph",
                                   PIPELINE, 3, main_extract);


}

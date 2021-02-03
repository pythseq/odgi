#include <path_sgd.hpp>
#include "subcommand.hpp"
#include "odgi.hpp"
#include "args.hxx"
#include "algorithms/extract/region.hpp"

namespace odgi {

    using namespace odgi::subcommand;

    //todo?
    void algorithms_extract_path_range(const PathPositionHandleGraph &source, path_handle_t path_handle, int64_t start,
                                       int64_t end, MutableHandleGraph &subgraph) {
        step_handle_t start_step = source.get_step_at_position(path_handle, start);
        size_t start_position = source.get_position_of_step(start_step);
        size_t size_needed = end < 0 ? numeric_limits<size_t>::max() : end - start + 1 + start - start_position;
        size_t running_length = 0;

        for (step_handle_t cur_step = start_step;
             cur_step != source.path_end(path_handle) && running_length < size_needed;
             cur_step = source.get_next_step(cur_step)) {
            handle_t cur_handle = source.get_handle_of_step(cur_step);
            subgraph.create_handle(source.get_sequence(cur_handle), source.get_id(cur_handle));
            if (cur_step != start_step) {
                handle_t prev_handle = source.get_handle_of_step(source.get_previous_step(cur_step));
                subgraph.create_edge(
                        subgraph.get_handle(source.get_id(prev_handle), source.get_is_reverse(prev_handle)),
                        subgraph.get_handle(source.get_id(cur_handle), source.get_is_reverse(cur_handle)));
            }
            running_length += source.get_length(cur_handle);
        }
    }

    //todo?
    void algorithms_expand_subgraph_by_length(const HandleGraph &source, MutableHandleGraph &subgraph,
                                              const uint64_t &length, bool forward_only) {
        uint64_t accumulated_length = 0;
        std::vector<handle_t> curr_handles;
        subgraph.for_each_handle([&](const handle_t &h) {
            curr_handles.push_back(h);
        });
        while (accumulated_length < length && !curr_handles.empty()) {
            std::vector<handle_t> next_handles;
            for (auto &h : curr_handles) {
                handle_t old_h = source.get_handle(subgraph.get_id(h));
                source.follow_edges(old_h, false, [&](const handle_t &c) {
                    handle_t x;
                    if (!subgraph.has_node(source.get_id(c))) {
                        x = subgraph.create_handle(source.get_sequence(source.get_is_reverse(c) ? source.flip(c) : c),
                                                   source.get_id(c));
                        next_handles.push_back(x);
                        accumulated_length += subgraph.get_length(x);
                    } else {
                        x = subgraph.get_handle(source.get_id(c));
                    }
                    if (source.get_is_reverse(c)) {
                        x = subgraph.flip(x);
                    }
                    if (!subgraph.has_edge(h, x)) {
                        subgraph.create_edge(h, x);
                    }
                });
                if (!forward_only) {
                    source.follow_edges(old_h, true, [&](const handle_t &c) {
                        handle_t x;
                        if (!subgraph.has_node(source.get_id(c))) {
                            x = subgraph.create_handle(
                                    source.get_sequence(source.get_is_reverse(c) ? source.flip(c) : c),
                                    source.get_id(c));
                            next_handles.push_back(x);
                            accumulated_length += subgraph.get_length(x);
                        } else {
                            x = subgraph.get_handle(source.get_id(c));
                        }
                        if (source.get_is_reverse(c)) {
                            x = subgraph.flip(x);
                        }
                        if (!subgraph.has_edge(x, h)) {
                            subgraph.create_edge(x, h);
                        }
                    });
                }
            }
            curr_handles = std::move(next_handles);
        }
        add_connecting_edges_to_subgraph(source, subgraph);
    }

    //todo?
    void
    algorithms_expand_subgraph_by_steps(const HandleGraph &source, MutableHandleGraph &subgraph, const uint64_t &steps,
                                        bool forward_only) {
        std::vector<handle_t> curr_handles;
        subgraph.for_each_handle([&](const handle_t &h) {
            curr_handles.push_back(h);
        });
        for (uint64_t i = 0; i < steps && !curr_handles.empty(); ++i) {
            std::vector<handle_t> next_handles;
            for (auto &h : curr_handles) {
                handle_t old_h = source.get_handle(subgraph.get_id(h));
                source.follow_edges(old_h, false, [&](const handle_t &c) {
                    handle_t x;
                    if (!subgraph.has_node(source.get_id(c))) {
                        x = subgraph.create_handle(source.get_sequence(source.get_is_reverse(c) ? source.flip(c) : c),
                                                   source.get_id(c));
                        next_handles.push_back(x);
                    } else {
                        x = subgraph.get_handle(source.get_id(c));
                    }
                    if (source.get_is_reverse(c)) {
                        x = subgraph.flip(x);
                    }
                    if (!subgraph.has_edge(h, x)) {
                        subgraph.create_edge(h, x);
                    }
                });
                if (!forward_only) {
                    source.follow_edges(old_h, true, [&](const handle_t &c) {
                        handle_t x;
                        if (!subgraph.has_node(source.get_id(c))) {
                            x = subgraph.create_handle(
                                    source.get_sequence(source.get_is_reverse(c) ? source.flip(c) : c),
                                    source.get_id(c));
                            next_handles.push_back(x);
                        } else {
                            x = subgraph.get_handle(source.get_id(c));
                        }
                        if (source.get_is_reverse(c)) {
                            x = subgraph.flip(x);
                        }
                        if (!subgraph.has_edge(x, h)) {
                            subgraph.create_edge(x, h);
                        }
                    });
                }
            }
            curr_handles = std::move(next_handles);
        }
        add_connecting_edges_to_subgraph(source, subgraph);
    }

    //todo?
    /// We can accumulate a subgraph without accumulating all the edges between its nodes
    /// this helper ensures that we get the full set
    void algorithms_add_connecting_edges_to_subgraph(const HandleGraph &source, MutableHandleGraph &subgraph) {
        subgraph.for_each_handle([&](const handle_t &handle) {
            nid_t id = subgraph.get_id(handle);
            handle_t source_handle = source.get_handle(id, subgraph.get_is_reverse(handle));
            source.follow_edges(source_handle, false, [&](const handle_t &next) {
                nid_t next_id = source.get_id(next);
                if (subgraph.has_node(next_id)) {
                    handle_t subgraph_next = subgraph.get_handle(next_id, source.get_is_reverse(next));
                    if (!subgraph.has_edge(handle, subgraph_next)) {
                        subgraph.create_edge(handle, subgraph_next);
                    }
                }
            });
            source.follow_edges(source_handle, true, [&](const handle_t &prev) {
                nid_t prev_id = source.get_id(prev);
                if (subgraph.has_node(prev_id)) {
                    handle_t subgraph_prev = subgraph.get_handle(prev_id, source.get_is_reverse(prev));
                    if (!subgraph.has_edge(subgraph_prev, handle)) {
                        subgraph.create_edge(subgraph_prev, handle);
                    }
                }
            });
        });
    }

    //todo?
    /// add subpaths to the subgraph, providing a concatenation of subpaths that are discontiguous over the subgraph
    /// based on their order in the path position index provided by the source graph
    /// will clear any path found in both graphs before writing the new steps into it
    /// if subpath_naming is true, a suffix will be added to each path in the subgraph denoting its offset
    /// in the source graph (unless the subpath was not cut up at all)
    void algorithms_add_subpaths_to_subgraph(const PathPositionHandleGraph &source, MutablePathHandleGraph &subgraph,
                                             bool subpath_naming) {
        std::unordered_map<std::string, std::map<uint64_t, handle_t> > subpaths;
        subgraph.for_each_handle([&](const handle_t &h) {
            handlegraph::nid_t id = subgraph.get_id(h);
            if (source.has_node(id)) {
                handle_t handle = source.get_handle(id);
                source.for_each_step_position_on_handle(handle, [&](const step_handle_t &step, const bool &is_rev,
                                                                    const uint64_t &pos) {
                    path_handle_t path = source.get_path_handle_of_step(step);
                    std::string path_name = source.get_path_name(path);
                    subpaths[path_name][pos] = is_rev ? subgraph.flip(h) : h;
                    return true;
                });
            }
        });

        function<path_handle_t(const string &, bool, size_t)> new_subpath =
                [&subgraph](const string &path_name, bool is_circular, size_t subpath_offset) {
                    string subpath_name = Paths::make_subpath_name(path_name, subpath_offset);
                    if (subgraph.has_path(subpath_name)) {
                        subgraph.destroy_path(subgraph.get_path_handle(subpath_name));
                    }
                    return subgraph.create_path_handle(subpath_name, is_circular);
                };

        for (auto &subpath : subpaths) {
            const std::string &path_name = subpath.first;
            path_handle_t source_path_handle = source.get_path_handle(path_name);
            // destroy the path if it exists
            if (subgraph.has_path(path_name)) {
                subgraph.destroy_path(subgraph.get_path_handle(path_name));
            }
            // create a new path.  give it a subpath name if the flag's on and its smaller than original
            path_handle_t path;
            if (!subpath_naming || subpath.second.size() == source.get_step_count(source_path_handle) ||
                subpath.second.empty()) {
                path = subgraph.create_path_handle(path_name, source.get_is_circular(source_path_handle));
            } else {
                path = new_subpath(path_name, source.get_is_circular(source_path_handle),
                                   subpath.second.begin()->first);
            }
            for (auto p = subpath.second.begin(); p != subpath.second.end(); ++p) {
                const handle_t &handle = p->second;
                if (p != subpath.second.begin() && subpath_naming) {
                    auto prev = p;
                    --prev;
                    const handle_t &prev_handle = prev->second;
                    // distance from map
                    size_t delta = p->first - prev->first;
                    // what the distance should be if they're contiguous depends on relative orienations
                    size_t cont_delta = subgraph.get_length(prev_handle);
                    if (delta != cont_delta) {
                        // we have a discontinuity!  we'll make a new path can continue from there
                        assert(subgraph.get_step_count(path) > 0);
                        path = new_subpath(path_name, subgraph.get_is_circular(path), p->first);
                    }
                }
                //fill in the path information
                subgraph.append_step(path, handle);
            }
        }
    }


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
        args::ValueFlag<std::string> _path_target_bed(parser, "FILE", "read the path targets from the given BED FILE",
                                                      {'P', "path-target-bed"});

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

        std::vector<Region> targets;

        // Given from the command line
        if (_path_target) {
            Region region;
            parse_region(args::get(_path_target), region);
            targets.push_back(region);
        }

        // Given from the BED file
        if (args::get(_path_target_bed).empty()) {
            parse_bed_regions(args::get(_path_target_bed), targets);
        }

        if (targets.empty()) {
            std::cerr
                    << "[odgi extract] error: please specify at least a target path to extract via -p=[STRING] or -P=[FILE]."
                    << std::endl;
            return 1;
        }

        xp::XP path_index;
        path_index.from_handle_graph(graph);

        graph_t subgraph;

        auto prep_graph = [&]() {
            if (context_size > 0) {
                if (use_length) {
                    algorithms_expand_subgraph_by_length(path_index, graph, context_size, false);
                } else {
                    algorithms_expand_subgraph_by_steps(path_index, graph, context_size, false);
                }
            } else {
                algorithms_add_connecting_edges_to_subgraph(path_index, graph);
            }

            algorithms_add_subpaths_to_subgraph(path_index, graph, false);

            //todo? subgraph.remove_orphan_edges();
            /*
            void remove_orphan_edges(Graph& graph) {
                set<id_t> ids;
                for (auto& node : graph.node()) {
                    ids.insert(node.id());
                }
                graph.mutable_edge()->erase(std::remove_if(graph.mutable_edge()->begin(),
                                                           graph.mutable_edge()->end(),
                                                           [&ids](const Edge& e) {
                                                               return !ids.count(e.from()) || !ids.count(e.to());
                                                           }), graph.mutable_edge()->end());
            }
             */

            // Order the mappings by rank. TODO: how do we handle breaks between
            // different sections of a path with a single name?
            // todo?subgraph.paths.sort_by_mapping_rank();
            /*
            // attempt to sort the paths based on the recorded ranks of the mappings
            void Paths::sort_by_mapping_rank(void) {
                for (auto p = _paths.begin(); p != _paths.end(); ++p) {
                    list<mapping_t>& path = p->second;
                    path.sort([](const mapping_t& m1, const mapping_t& m2) {
                            return m1.rank < m2.rank;
                        });
                }
            }
             */
        };

        for (auto &target : targets) {
            // Grab each target region
            if (!path_index.has_path(target.seq)) {
                // Passing a nonexistent path to get_path_range produces Undefined Behavior
                cerr << "[odgi extract] error, path " << target.seq << " not found in index" << endl;
                exit(1);
            }
            path_handle_t path_handle = path_index.get_path_handle(target.seq);
            // no coordinates given, we do whole thing (0,-1)
            if (target.start < 0 && target.end < 0) {
                target.start = 0;
            }
            algorithms_extract_path_range(path_index, path_handle, target.start, target.end, graph);

            /*if (!save_to_prefix.empty()) {
                prep_graph();
                // write to our save_to file
                stringstream s;
                s << save_to_prefix << target.seq;
                if (target.end >= 0) s << ":" << target.start << ":" << target.end;
                s << ".vg";
                ofstream out(s.str().c_str());
                graph.serialize(std::cout);
                out.close();
                // reset our graph
                VG empty;
                graph = empty;
            }*/

        }

        prep_graph();
        graph.serialize(std::cout);

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

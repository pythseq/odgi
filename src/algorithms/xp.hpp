#pragma once

#include "sdsl/bit_vectors.hpp"
#include "sdsl/enc_vector.hpp"
#include "sdsl/dac_vector.hpp"
#include "sdsl/vlc_vector.hpp"
#include "sdsl/wavelet_trees.hpp"
#include "sdsl/csa_wt.hpp"
#include "sdsl/suffix_arrays.hpp"

#include <handlegraph/types.hpp>
#include <handlegraph/iteratee.hpp>
#include <handlegraph/util.hpp>
#include <handlegraph/handle_graph.hpp>
#include <handlegraph/path_position_handle_graph.hpp>
#include <handlegraph/serializable_handle_graph.hpp>

namespace xp {

    using nid_t = handlegraph::nid_t;

    class XPPath;

    /**
 * Thrown when attempting to interpret invalid data as an XP index.
 */
    class XPFormatError : public std::runtime_error {
        // Use the runtime_error constructor
        using std::runtime_error::runtime_error;
    };
 /**
 * Provides succinct storage for the positional paths of a graph.
 */
    // class XP : public handlegraph::PathPositionHandleGraph, public handlegraph::SerializableHandleGraph, public handlegraph::VectorizableHandleGraph {
    // TODO @ekg Do I need any of these abstract classes? I would have to implement a huge amount of functions. See path_handle_graph.hpp for example.
    class XP {
    public:

        ////////////////////////////////////////////////////////////////////////////
        // Here are the ways we can construct an XP object from a graph
        ////////////////////////////////////////////////////////////////////////////

        XP(void) = default;

        ~XP(void);

        // We cannot move, assign, or copy until we add code to point sdsl supports
        // at the new addresses for their vectors.
        XP(const XP &other) = delete;

        XP(XP&& other) = delete;

        XP &operator=(const XP &other) = delete;

        XP &operator=(XP &&other) = delete;

        // General public statistics
        size_t seq_length = 0;
        size_t node_count = 0;
        size_t edge_count = 0;
        size_t path_count = 0;

        // Build the path index from a simple graph.
        void from_handle_graph(const handlegraph::HandleGraph &graph);

        // TODO Do we want to support from GFA?

        // Get our magic number
        // TODO @ekg Do we need this? How would it look like?
        uint32_t get_magic_number(void) const;

        // Load this XP index from a stream. Throw an XPFormatError if the stream
        // does not produce a valid XP file.
        void load(std::istream& in);

        // Alias for load() to match the SerializableHandleGraph interface.
        void deserialize_members(std::istream& in);

        // Write this XP index to a stream.
        size_t serialize_and_measure(std::ostream& out, sdsl::structure_tree_node* s = nullptr, std::string name = "") const;
        // Alias for serialize_and_measure().
        void serialize_members(std::ostream& out) const;

        // For a given path name and nucleotide position return its rank
        size_t path_pos_to_rank(std::string& path_name, uint64_t pos); // TODO @ekg Or is it size_t here?
        // TODO We might need some helper functions for the above

        // TODO We need to store our XPPaths somewhere

    private:
        ////////////////////////////////////////////////////////////////////////////
        // Here is path storage
        ////////////////////////////////////////////////////////////////////////////

        // path names
        sdsl::int_vector<> pn_iv; // path names
        sdsl::csa_wt<> pn_csa; // path name compressed suffix array
        sdsl::bit_vector pn_bv;  // path name starts in uncompressed version of csa
        sdsl::rank_support_v<1> pn_bv_rank;
        sdsl::bit_vector::select_1_type pn_bv_select;
        sdsl::int_vector<> pi_iv; // path ids by rank in the path names

        std::vector<XPPath*> paths; // path structure

        // TODO @ekg I won't need any of this, right?
        /**
        // node->path membership
        sdsl::int_vector<> np_iv;
        // node->path rank
        sdsl::int_vector<> nr_iv;
        // node->path position/orientation
        sdsl::int_vector<> nx_iv;
        sdsl::bit_vector np_bv; // entity delimiters in both vectors
        //sdsl::rank_support_v<1> np_bv_rank;
        sdsl::bit_vector::select_1_type np_bv_select;
        **/
    };

    class XPPath {
    public:
        XPPath(void) = default;
        ~XPPath(void) = default;
        // Path name is required here only for complaining intelligently when
        // something goes wrong. We can also spit out the total unique members,
        // because in here is the most efficient place to count them.
        XPPath(const std::string& path_name,
               const std::vector<handlegraph::handle_t>& path,
               bool is_circular,
               XP& graph);
        // Path names are stored in the XP object, in a compressed fashion, and are
        // not duplicated here.

        // These contain rank and select supports and so cannot move or be copied
        // without code to update them.
        XPPath(const XPPath& other) = delete;
        XPPath(XPPath&& other) = delete;
        XPPath& operator=(const XPPath& other) = delete;
        XPPath& operator=(XPPath&& other) = delete;
        handlegraph::handle_t min_handle;
        // TODO @ekg Do I need all of them? As far as I got it I don't need offsets?
        sdsl::enc_vector<> handles;
        //sdsl::rrr_vector directions; // forward or backward through nodes
        sdsl::rrr_vector<> offsets;
        sdsl::rrr_vector<>::rank_1_type offsets_rank;
        sdsl::rrr_vector<>::select_1_type offsets_select;
        // TODO €ekg Do we need that?
        bool is_circular = false;

        void load(std::istream& in);

        size_t serialize(std::ostream& out,
                         sdsl::structure_tree_node* v = NULL,
                         std::string name = "") const;

        size_t step_rank_at_position(size_t pos) const;

        // TODO @ekg do I need more functionality here?
    };
}

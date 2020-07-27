// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_AIS_HYPER_GRAPH_HH
#define G2O_AIS_HYPER_GRAPH_HH

#include <map>
#include <set>
#include <bitset>
#include <cassert>
#include <vector>
#include <limits>
#include <cstddef>

#if defined(_MSC_VER) || defined(__APPLE__)
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif


/** @addtogroup graph */
//@{
namespace g2o {

  /**
     Class that models a directed  Hyper-Graph. An hyper graph is a graph where an edge
     can connect one or more nodes. Both Vertices and Edges of an hyoper graph
     derive from the same class HyperGraphElement, thus one can implement generic algorithms
     that operate transparently on edges or vertices (see HyperGraphAction).

     The vertices are uniquely identified by an int id, while the edges are
     identfied by their pointers. 
   */
  class  HyperGraph
  {
    public:

      /**
       * \brief enum of all the types we have in our graphs
       */
      enum  HyperGraphElementType {
        HGET_VERTEX,
        HGET_EDGE,
        HGET_PARAMETER,
        HGET_CACHE,
        HGET_DATA,
        HGET_NUM_ELEMS // keep as last elem
      };

      typedef std::bitset<HyperGraph::HGET_NUM_ELEMS> GraphElemBitset;

      class  Vertex;
      class  Edge;
      
      /**
       * base hyper graph element, specialized in vertex and edge
       */
      struct  HyperGraphElement {
        virtual ~HyperGraphElement() {}
        /**
         * returns the type of the graph element, see HyperGraphElementType
         */
        virtual HyperGraphElementType elementType() const = 0;
      };

      typedef std::set<Edge*>                           EdgeSet;
      typedef std::set<Vertex*>                         VertexSet;

      #if defined(_MSC_VER) || defined(__APPLE__)
      typedef std::unordered_map<int, Vertex*>          VertexIDMap;
      #else
      typedef std::tr1::unordered_map<int, Vertex*>     VertexIDMap;
      #endif
      typedef std::vector<Vertex*>                      VertexContainer;

      //! abstract Vertex, your types must derive from that one
      class  Vertex : public HyperGraphElement {
        public:
          //! creates a vertex having an ID specified by the argument
          explicit Vertex(int id=-1);
          virtual ~Vertex();
          //! returns the id
          int id() const {return _id;}
	  virtual void setId( int newId) { _id=newId; }
          //! returns the set of hyper-edges that are leaving/entering in this vertex
          const EdgeSet& edges() const {return _edges;}
          //! returns the set of hyper-edges that are leaving/entering in this vertex
          EdgeSet& edges() {return _edges;}
          virtual HyperGraphElementType elementType() const { return HGET_VERTEX;}
        protected:
          int _id;
          EdgeSet _edges;
      };

      /** 
       * Abstract Edge class. Your nice edge classes should inherit from that one.
       * An hyper-edge has pointers to the vertices it connects and stores them in a vector.
       */
      class  Edge : public HyperGraphElement {
        public:
          //! creates and empty edge with no vertices
          explicit Edge(int id = -1);
          virtual ~Edge();

          /**
           * resizes the number of vertices connected by this edge
           */
          virtual void resize(size_t size);
          /**
            returns the vector of pointers to the vertices connected by the hyper-edge.
            */
          const VertexContainer& vertices() const { return _vertices;}
          /**
            returns the vector of pointers to the vertices connected by the hyper-edge.
            */
          VertexContainer& vertices() { return _vertices;}
          /**
            returns the pointer to the ith vertex connected to the hyper-edge.
            */
          const Vertex* vertex(size_t i) const { assert(i < _vertices.size() && "index out of bounds"); return _vertices[i];}
          /**
            returns the pointer to the ith vertex connected to the hyper-edge.
            */
          Vertex* vertex(size_t i) { assert(i < _vertices.size() && "index out of bounds"); return _vertices[i];}
          /**
            set the ith vertex on the hyper-edge to the pointer supplied
            */
          void setVertex(size_t i, Vertex* v) { assert(i < _vertices.size() && "index out of bounds"); _vertices[i]=v;}

          int id() const {return _id;}
          void setId(int id);
          virtual HyperGraphElementType elementType() const { return HGET_EDGE;}
        protected:
          VertexContainer _vertices;
          int _id; ///< unique id
      };

    public:
      //! constructs an empty hyper graph
      HyperGraph();
      //! destroys the hyper-graph and all the vertices of the graph
      virtual ~HyperGraph();

      //! returns a vertex <i>id</i> in the hyper-graph, or 0 if the vertex id is not present
      Vertex* vertex(int id);
      //! returns a vertex <i>id</i> in the hyper-graph, or 0 if the vertex id is not present
      const Vertex* vertex(int id) const;

      //! removes a vertex from the graph. Returns true on success (vertex was present)
      virtual bool removeVertex(Vertex* v);
      //! removes a vertex from the graph. Returns true on success (edge was present)
      virtual bool removeEdge(Edge* e);
      //! clears the graph and empties all structures.
      virtual void clear();

      //! @returns the map <i>id -> vertex</i> where the vertices are stored
      const VertexIDMap& vertices() const {return _vertices;}
      //! @returns the map <i>id -> vertex</i> where the vertices are stored
      VertexIDMap& vertices() {return _vertices;}

      //! @returns the set of edges of the hyper graph
      const EdgeSet& edges() const {return _edges;}
      //! @returns the set of edges of the hyper graph
      EdgeSet& edges() {return _edges;}

      /**
       * adds a vertex to the graph. The id of the vertex should be set before
       * invoking this function. the function fails if another vertex
       * with the same id is already in the graph.
       * returns true, on success, or false on failure.
       */
      virtual bool addVertex(Vertex* v);

      /**
       * Adds an edge  to the graph. If the edge is already in the graph, it
       * does nothing and returns false. Otherwise it returns true.
       */
      virtual bool addEdge(Edge* e);

      /**
       * changes the id of a vertex already in the graph, and updates the bookkeeping
       @ returns false if the vertex is not in the graph;
       */
      virtual bool changeId(Vertex* v, int newId);

    protected:
      VertexIDMap _vertices;
      EdgeSet _edges;

    private:
      // Disable the copy constructor and assignment operator
      HyperGraph(const HyperGraph&) { }
      HyperGraph& operator= (const HyperGraph&) { return *this; }
  };

} // end namespace

//@}

#endif

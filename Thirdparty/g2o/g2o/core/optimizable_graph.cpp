// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "optimizable_graph.h"

#include <cassert>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

#include <eigen3/Eigen/Dense>

#include "estimate_propagator.h"
#include "factory.h"
#include "optimization_algorithm_property.h"
#include "hyper_graph_action.h"
#include "cache.h"
#include "robust_kernel.h"

#include "../stuff/macros.h"
#include "../stuff/color_macros.h"
#include "../stuff/string_tools.h"
#include "../stuff/misc.h"

namespace g2o {

  using namespace std;

  OptimizableGraph::Data::Data(){
    _next = 0;
  }
  
  OptimizableGraph::Data::~Data(){
    if (_next)
      delete _next;
  }
  

  OptimizableGraph::Vertex::Vertex() :
    HyperGraph::Vertex(),
    _graph(0), _userData(0), _hessianIndex(-1), _fixed(false), _marginalized(false),
    _colInHessian(-1), _cacheContainer(0)
  {
  }

  CacheContainer* OptimizableGraph::Vertex::cacheContainer(){
    if (! _cacheContainer)
      _cacheContainer = new CacheContainer(this);
    return _cacheContainer;
  }


  void OptimizableGraph::Vertex::updateCache(){
    if (_cacheContainer){
      _cacheContainer->setUpdateNeeded();
      _cacheContainer->update();
    }
  }

  OptimizableGraph::Vertex::~Vertex()
  {
    if (_cacheContainer)
      delete (_cacheContainer);
    if (_userData)
      delete _userData;
  }
  
  OptimizableGraph::Vertex* OptimizableGraph::Vertex::clone() const
  {
    return 0;
  }

  bool OptimizableGraph::Vertex::setEstimateData(const double* v)
  {
    bool ret = setEstimateDataImpl(v);
    updateCache();
    return ret;
  }

  bool OptimizableGraph::Vertex::getEstimateData(double *) const
  {
    return false;
  }

  int OptimizableGraph::Vertex::estimateDimension() const
  {
    return -1;
  }

  bool OptimizableGraph::Vertex::setMinimalEstimateData(const double* v)
  {
    bool ret = setMinimalEstimateDataImpl(v);
    updateCache();
    return ret;
  }

  bool OptimizableGraph::Vertex::getMinimalEstimateData(double *) const
  {
    return false;
  }

  int OptimizableGraph::Vertex::minimalEstimateDimension() const
  {
    return -1;
  }


  OptimizableGraph::Edge::Edge() :
    HyperGraph::Edge(),
    _dimension(-1), _level(0), _robustKernel(0)
  {
  }

  OptimizableGraph::Edge::~Edge()
  {
    delete _robustKernel;
  }

  OptimizableGraph* OptimizableGraph::Edge::graph(){
    if (! _vertices.size())
      return 0;
    OptimizableGraph::Vertex* v=(OptimizableGraph::Vertex*)_vertices[0];
    if (!v)
      return 0;
    return v->graph();
  }
  
  const OptimizableGraph* OptimizableGraph::Edge::graph() const{
    if (! _vertices.size())
      return 0;
    const OptimizableGraph::Vertex* v=(const OptimizableGraph::Vertex*) _vertices[0];
    if (!v)
      return 0;
    return v->graph();
  }

  bool OptimizableGraph::Edge::setParameterId(int argNum, int paramId){
    if ((int)_parameters.size()<=argNum)
      return false;
    if (argNum<0)
      return false;
    *_parameters[argNum] = 0;
    _parameterIds[argNum] = paramId;
    return true;
  }

  bool OptimizableGraph::Edge::resolveParameters() {
    if (!graph()) {
      cerr << __PRETTY_FUNCTION__ << ": edge not registered with a graph" << endl;
      return false;
    }
    
    assert (_parameters.size() == _parameterIds.size());
    //cerr << __PRETTY_FUNCTION__ << ": encountered " << _parameters.size() << " parameters" << endl;
    for (size_t i=0; i<_parameters.size(); i++){
      int index = _parameterIds[i];
      *_parameters[i] = graph()->parameter(index);
      if (typeid(**_parameters[i]).name()!=_parameterTypes[i]){
        cerr << __PRETTY_FUNCTION__ << ": FATAL, parameter type mismatch - encountered " << typeid(**_parameters[i]).name() << "; should be " << _parameterTypes[i] << endl;
      }
      if (!*_parameters[i]) {
        cerr << __PRETTY_FUNCTION__ << ": FATAL, *_parameters[i] == 0" << endl;
        return false;
      }
    }
    return true;
  }

  void OptimizableGraph::Edge::setRobustKernel(RobustKernel* ptr)
  {
    if (_robustKernel)
      delete _robustKernel;
    _robustKernel = ptr;
  }

  bool OptimizableGraph::Edge::resolveCaches() {
    return true;
  }

  bool OptimizableGraph::Edge::setMeasurementData(const double *)
  {
    return false;
  }

  bool OptimizableGraph::Edge::getMeasurementData(double *) const
  {
    return false;
  }

  int OptimizableGraph::Edge::measurementDimension() const
  {
    return -1;
  }

  bool OptimizableGraph::Edge::setMeasurementFromState(){
    return false;
  }


  OptimizableGraph::Edge* OptimizableGraph::Edge::clone() const
  {
    // TODO
    return 0;
  }


  OptimizableGraph::OptimizableGraph()
  {
    _nextEdgeId = 0; _edge_has_id = false;
    _graphActions.resize(AT_NUM_ELEMENTS);
  }

  OptimizableGraph::~OptimizableGraph()
  {
    clear();
    clearParameters();
  }

  bool OptimizableGraph::addVertex(HyperGraph::Vertex* v, Data* userData)
  {
    Vertex* inserted = vertex(v->id());
    if (inserted) {
      cerr << __FUNCTION__ << ": FATAL, a vertex with ID " << v->id() << " has already been registered with this graph" << endl;
      assert(0 && "Vertex with this ID already contained in the graph");
      return false;
    }
    OptimizableGraph::Vertex* ov=dynamic_cast<OptimizableGraph::Vertex*>(v);
    assert(ov && "Vertex does not inherit from OptimizableGraph::Vertex");
    if (ov->_graph != 0 && ov->_graph != this) {
      cerr << __FUNCTION__ << ": FATAL, vertex with ID " << v->id() << " has already registered with another graph " << ov->_graph << endl;
      assert(0 && "Vertex already registered with another graph");
      return false;
    }
    if (userData)
      ov->setUserData(userData);
    ov->_graph=this;
    return HyperGraph::addVertex(v);
  }

  bool OptimizableGraph::addEdge(HyperGraph::Edge* e_)
  {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(e_);
    assert(e && "Edge does not inherit from OptimizableGraph::Edge");
    if (! e)
      return false;
    bool eresult = HyperGraph::addEdge(e);
    if (! eresult)
      return false;
    e->_internalId = _nextEdgeId++;
    if (! e->resolveParameters()){
      cerr << __FUNCTION__ << ": FATAL, cannot resolve parameters for edge " << e << endl;
      return false;
    }
    if (! e->resolveCaches()){
      cerr << __FUNCTION__ << ": FATAL, cannot resolve caches for edge " << e << endl;
      return false;
    } 
    _jacobianWorkspace.updateSize(e);

    return true;
  }

  int OptimizableGraph::optimize(int /*iterations*/, bool /*online*/) {return 0;}

double OptimizableGraph::chi2() const
{
  double chi = 0.0;
  for (OptimizableGraph::EdgeSet::const_iterator it = this->edges().begin(); it != this->edges().end(); ++it) {
    const OptimizableGraph::Edge* e = static_cast<const OptimizableGraph::Edge*>(*it);
    chi += e->chi2();
  }
  return chi;
}

void OptimizableGraph::push()
{
  for (OptimizableGraph::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
    v->push();
  }
}

void OptimizableGraph::pop()
{
  for (OptimizableGraph::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it) {
    OptimizableGraph::Vertex* v= static_cast<OptimizableGraph::Vertex*>(it->second);
    v->pop();
  }
}

void OptimizableGraph::discardTop()
{
  for (OptimizableGraph::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it) {
    OptimizableGraph::Vertex* v= static_cast<OptimizableGraph::Vertex*>(it->second);
    v->discardTop();
  }
}

void OptimizableGraph::push(HyperGraph::VertexSet& vset)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->push();
  }
}

void OptimizableGraph::pop(HyperGraph::VertexSet& vset)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->pop();
  }
}

void OptimizableGraph::discardTop(HyperGraph::VertexSet& vset)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->discardTop();
  }
}

  void OptimizableGraph::setFixed(HyperGraph::VertexSet& vset, bool fixed)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->setFixed(fixed);
  }
}


bool OptimizableGraph::load(istream& is, bool createEdges)
{
  // scna for the paramers in the whole file
  if (!_parameters.read(is,&_renamedTypesLookup))
    return false;
#ifndef NDEBUG
  cerr << "Loaded " << _parameters.size() << " parameters" << endl;
#endif
  is.clear();
  is.seekg(ios_base::beg);
  set<string> warnedUnknownTypes;
  stringstream currentLine;
  string token;

  Factory* factory = Factory::instance();
  HyperGraph::GraphElemBitset elemBitset;
  elemBitset[HyperGraph::HGET_PARAMETER] = 1;
  elemBitset.flip();

  Vertex* previousVertex = 0;
  Data* previousData = 0;

  while (1) {
    int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1)
      break;
    currentLine >> token;
    //cerr << "Token=" << token << endl;
    if (bytesRead == 0 || token.size() == 0 || token[0] == '#')
      continue;

    // handle commands encoded in the file
    bool handledCommand = false;
    
    if (token == "FIX") {
      handledCommand = true;
      int id;
      while (currentLine >> id) {
        OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(vertex(id));
        if (v) {
#        ifndef NDEBUG
          cerr << "Fixing vertex " << v->id() << endl;
#        endif
          v->setFixed(true);
        } else {
          cerr << "Warning: Unable to fix vertex with id " << id << ". Not found in the graph." << endl;
        }
      }
    }

    if (handledCommand)
      continue;
     
    // do the mapping to an internal type if it matches
    if (_renamedTypesLookup.size() > 0) {
      map<string, string>::const_iterator foundIt = _renamedTypesLookup.find(token);
      if (foundIt != _renamedTypesLookup.end()) {
        token = foundIt->second;
      }
    }

    if (! factory->knowsTag(token)) {
      if (warnedUnknownTypes.count(token) != 1) {
        warnedUnknownTypes.insert(token);
        cerr << CL_RED(__PRETTY_FUNCTION__ << " unknown type: " << token) << endl;
      }
      continue;
    }

    HyperGraph::HyperGraphElement* element = factory->construct(token, elemBitset);
    if (dynamic_cast<Vertex*>(element)) { // it's a vertex type
      //cerr << "it is a vertex" << endl;
      previousData = 0;
      Vertex* v = static_cast<Vertex*>(element);
      int id;
      currentLine >> id;
      bool r = v->read(currentLine);
      if (! r)
        cerr << __PRETTY_FUNCTION__ << ": Error reading vertex " << token << " " << id << endl;
      v->setId(id);
      if (!addVertex(v)) {
        cerr << __PRETTY_FUNCTION__ << ": Failure adding Vertex, " << token << " " << id << endl;
        delete v;
      } else {
        previousVertex = v;
      }
    }
    else if (dynamic_cast<Edge*>(element)) {
      //cerr << "it is an edge" << endl;
      previousData = 0;
      Edge* e = static_cast<Edge*>(element);
      int numV = e->vertices().size();
      if (_edge_has_id){
        int id;
        currentLine >> id;
        e->setId(id);
      }
      //cerr << PVAR(token) << " " << PVAR(numV) << endl;
      if (numV == 2) { // it's a pairwise / binary edge type which we handle in a special way
        int id1, id2;
        currentLine >> id1 >> id2;
        Vertex* from = vertex(id1);
        Vertex* to = vertex(id2);
        int doInit=0;
        if ((!from || !to) ) {
          if (! createEdges) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to find vertices for edge " << token << " " << id1 << " <-> " << id2 << endl;
            delete e;
          } else {
            if (! from) {
              from=e->createFrom();
              from->setId(id1);
              addVertex(from);
              doInit=2;
            }
            if (! to) {
              to=e->createTo();
              to->setId(id2);
              addVertex(to);
              doInit=1;
            }
          }
        }
        if (from && to) {
          e->setVertex(0, from);
          e->setVertex(1, to);
          e->read(currentLine);
          if (!addEdge(e)) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to add edge " << token << " " << id1 << " <-> " << id2 << endl;
            delete e;
          } else {
            switch (doInit){
              case 1: 
                {
                  HyperGraph::VertexSet fromSet;
                  fromSet.insert(from);
                  e->initialEstimate(fromSet, to);
                  break;
                }
              case 2:
                {
                  HyperGraph::VertexSet toSet;
                  toSet.insert(to);
                  e->initialEstimate(toSet, from);
                  break;
                }
              default:;
            }
          }
        }
      }
      else {
        vector<int> ids;
        ids.resize(numV);
        for (int l = 0; l < numV; ++l)
          currentLine >> ids[l];
        bool vertsOkay = true;
        for (int l = 0; l < numV; ++l) {
          e->setVertex(l, vertex(ids[l]));
          if (e->vertex(l) == 0) {
            vertsOkay = false;
            break;
          }
        }
        if (! vertsOkay) {
          cerr << __PRETTY_FUNCTION__ << ": Unable to find vertices for edge " << token;
          for (int l = 0; l < numV; ++l) {
            if (l > 0)
              cerr << " <->";
            cerr << " " << ids[l];
          }
          delete e;
        } else {
          bool r = e->read(currentLine);
          if (!r || !addEdge(e)) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to add edge " << token; 
            for (int l = 0; l < numV; ++l) {
              if (l > 0)
                cerr << " <->";
              cerr << " " << ids[l];
            }
            delete e;
          }
        }
      }
    } else if (dynamic_cast<Data*>(element)) { // reading in the data packet for the vertex
      //cerr << "read data packet " << token << " vertex " << previousVertex->id() << endl;
      Data* d = static_cast<Data*>(element);
      bool r = d->read(currentLine);
      if (! r) {
	cerr << __PRETTY_FUNCTION__ << ": Error reading data " << token << " for vertex " << previousVertex->id() << endl;
	delete d;
	previousData = 0;
      } else if (previousData){
	//cerr << "chaining" << endl;
	previousData->setNext(d);
        previousData = d;
	//cerr << "done" << endl;
      } else if (previousVertex){
	//cerr << "embedding in vertex" << endl;
	previousVertex->setUserData(d);
	previousData = d;
	previousVertex = 0;
	//cerr << "done" << endl;
      } else {
        cerr << __PRETTY_FUNCTION__ << ": got data element, but no vertex available" << endl;
        delete d;
	previousData = 0;
      }
    }
  } // while read line
  
  return true;
}

bool OptimizableGraph::load(const char* filename, bool createEdges)
{
  ifstream ifs(filename);
  if (!ifs) {
    cerr << __PRETTY_FUNCTION__ << " unable to open file " << filename << endl;
    return false;
  }
  return load(ifs, createEdges);
}

bool OptimizableGraph::save(const char* filename, int level) const
{
  ofstream ofs(filename);
  if (!ofs)
    return false;
  return save(ofs, level);
}

bool OptimizableGraph::save(ostream& os, int level) const
{
  if (! _parameters.write(os))
    return false;
  set<Vertex*, VertexIDCompare> verticesToSave;
  for (HyperGraph::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    if (e->level() == level) {
      for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
        verticesToSave.insert(static_cast<OptimizableGraph::Vertex*>(*it));
      }
    }
  }

  for (set<Vertex*, VertexIDCompare>::const_iterator it = verticesToSave.begin(); it != verticesToSave.end(); ++it){
    OptimizableGraph::Vertex* v = *it;
    saveVertex(os, v);
  }

  EdgeContainer edgesToSave;
  for (HyperGraph::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it) {
    const OptimizableGraph::Edge* e = dynamic_cast<const OptimizableGraph::Edge*>(*it);
    if (e->level() == level)
      edgesToSave.push_back(const_cast<Edge*>(e));
  }
  sort(edgesToSave.begin(), edgesToSave.end(), EdgeIDCompare());

  for (EdgeContainer::const_iterator it = edgesToSave.begin(); it != edgesToSave.end(); ++it) {
    OptimizableGraph::Edge* e = *it;
    saveEdge(os, e);
  }

  return os.good();
}


bool OptimizableGraph::saveSubset(ostream& os, HyperGraph::VertexSet& vset, int level)
{
  if (! _parameters.write(os))
    return false;

  for (HyperGraph::VertexSet::const_iterator it=vset.begin(); it!=vset.end(); it++){
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(*it);
    saveVertex(os, v);
  }
  for (HyperGraph::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = dynamic_cast< OptimizableGraph::Edge*>(*it);
    if (e->level() != level)
      continue;

    bool verticesInEdge = true;
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      if (vset.find(*it) == vset.end()) {
        verticesInEdge = false;
        break;
      }
    }
    if (! verticesInEdge)
      continue;

    saveEdge(os, e);
  }

  return os.good();
}

bool OptimizableGraph::saveSubset(ostream& os, HyperGraph::EdgeSet& eset)
{
  if (!_parameters.write(os))
    return false;
  std::set<OptimizableGraph::Vertex*> vset;
  for (HyperGraph::EdgeSet::const_iterator it = eset.begin(); it != eset.end(); ++it) {
    HyperGraph::Edge* e = *it;
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
      vset.insert(v);
    }
  }

  for (std::set<OptimizableGraph::Vertex*>::const_iterator it=vset.begin(); it!=vset.end(); ++it){
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(*it);
    saveVertex(os, v);
  }

  for (HyperGraph::EdgeSet::const_iterator it = eset.begin(); it != eset.end(); ++it) {
    OptimizableGraph::Edge* e = dynamic_cast< OptimizableGraph::Edge*>(*it);
    saveEdge(os, e);
  }

  return os.good();
}
  
void OptimizableGraph::addGraph(OptimizableGraph* g){
  for (HyperGraph::VertexIDMap::iterator it=g->vertices().begin(); it!=g->vertices().end(); ++it){
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(it->second);
    if (vertex(v->id()))
      continue;
    OptimizableGraph::Vertex* v2=v->clone();
    v2->edges().clear();
    v2->setHessianIndex(-1);
    addVertex(v2);
  }
  for (HyperGraph::EdgeSet::iterator it=g->edges().begin(); it!=g->edges().end(); ++it){
    OptimizableGraph::Edge* e = (OptimizableGraph::Edge*)(*it);
    OptimizableGraph::Edge* en = e->clone();
    en->resize(e->vertices().size());
    int cnt = 0;
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*) vertex((*it)->id());
      assert(v);
      en->setVertex(cnt++, v);
    }
    addEdge(en);
  }
}

int OptimizableGraph::maxDimension() const{
  int maxDim=0;
  for (HyperGraph::VertexIDMap::const_iterator it=vertices().begin(); it!=vertices().end(); ++it){
    const OptimizableGraph::Vertex* v= static_cast< const OptimizableGraph::Vertex*>(it->second);
    maxDim = (std::max)(maxDim, v->dimension());
  }
  return maxDim;
}

void OptimizableGraph::setRenamedTypesFromString(const std::string& types)
{
  Factory* factory = Factory::instance();
  vector<string> typesMap = strSplit(types, ",");
  for (size_t i = 0; i < typesMap.size(); ++i) {
    vector<string> m = strSplit(typesMap[i], "=");
    if (m.size() != 2) {
      cerr << __PRETTY_FUNCTION__ << ": unable to extract type map from " << typesMap[i] << endl;
      continue;
    }
    string typeInFile = trim(m[0]);
    string loadedType = trim(m[1]);
    if (! factory->knowsTag(loadedType)) {
      cerr << __PRETTY_FUNCTION__ << ": unknown type " << loadedType << endl;
      continue;
    }

    _renamedTypesLookup[typeInFile] = loadedType;
  }

  cerr << "# load look up table" << endl;
  for (std::map<std::string, std::string>::const_iterator it = _renamedTypesLookup.begin(); it != _renamedTypesLookup.end(); ++it) {
    cerr << "#\t" << it->first << " -> " << it->second << endl;
  }
}

bool OptimizableGraph::isSolverSuitable(const OptimizationAlgorithmProperty& solverProperty, const std::set<int>& vertDims_) const
{
  std::set<int> auxDims;
  if (vertDims_.size() == 0) {
    auxDims = dimensions();
  }
  const set<int>& vertDims = vertDims_.size() == 0 ? auxDims : vertDims_;
  bool suitableSolver = true;
  if (vertDims.size() == 2) {
    if (solverProperty.requiresMarginalize) {
      suitableSolver = vertDims.count(solverProperty.poseDim) == 1 && vertDims.count(solverProperty.landmarkDim) == 1;
    }
    else {
      suitableSolver = solverProperty.poseDim == -1;
    }
  } else if (vertDims.size() == 1) {
    suitableSolver = vertDims.count(solverProperty.poseDim) == 1 || solverProperty.poseDim == -1;
  } else {
    suitableSolver = solverProperty.poseDim == -1 && !solverProperty.requiresMarginalize;
  }
  return suitableSolver;
}

std::set<int> OptimizableGraph::dimensions() const
{
  std::set<int> auxDims;
  for (VertexIDMap::const_iterator it = vertices().begin(); it != vertices().end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
    auxDims.insert(v->dimension());
  }
  return auxDims;
}

void OptimizableGraph::preIteration(int iter)
{
  HyperGraphActionSet& actions = _graphActions[AT_PREITERATION];
  if (actions.size() > 0) {
    HyperGraphAction::ParametersIteration params(iter);
    for (HyperGraphActionSet::iterator it = actions.begin(); it != actions.end(); ++it) {
      (*(*it))(this, &params);
    }
  }
}

void OptimizableGraph::postIteration(int iter)
{
  HyperGraphActionSet& actions = _graphActions[AT_POSTITERATION];
  if (actions.size() > 0) {
    HyperGraphAction::ParametersIteration params(iter);
    for (HyperGraphActionSet::iterator it = actions.begin(); it != actions.end(); ++it) {
      (*(*it))(this, &params);
    }
  }
}

bool OptimizableGraph::addPostIterationAction(HyperGraphAction* action)
{
  std::pair<HyperGraphActionSet::iterator, bool> insertResult = _graphActions[AT_POSTITERATION].insert(action);
  return insertResult.second;
}

bool OptimizableGraph::addPreIterationAction(HyperGraphAction* action)
{
  std::pair<HyperGraphActionSet::iterator, bool> insertResult = _graphActions[AT_PREITERATION].insert(action);
  return insertResult.second;
}

bool OptimizableGraph::removePreIterationAction(HyperGraphAction* action)
{
  return _graphActions[AT_PREITERATION].erase(action) > 0;
}

bool OptimizableGraph::removePostIterationAction(HyperGraphAction* action)
{
  return _graphActions[AT_POSTITERATION].erase(action) > 0;
}

bool OptimizableGraph::saveVertex(std::ostream& os, OptimizableGraph::Vertex* v) const
{
  Factory* factory = Factory::instance();
  string tag = factory->tag(v);
  if (tag.size() > 0) {
    os << tag << " " << v->id() << " ";
    v->write(os);
    os << endl;
    Data* d=v->userData();
    while (d) { // write the data packet for the vertex
      tag = factory->tag(d);
      if (tag.size() > 0) {
        os << tag << " ";
        d->write(os);
        os << endl;
      }
      d=d->next();
    }
    if (v->fixed()) {
      os << "FIX " << v->id() << endl;
    }
    return os.good();
  }
  return false;
}

bool OptimizableGraph::saveEdge(std::ostream& os, OptimizableGraph::Edge* e) const
{
  Factory* factory = Factory::instance();
  string tag = factory->tag(e);
  if (tag.size() > 0) {
    os << tag << " ";
    if (_edge_has_id)
      os << e->id() << " ";
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
      os << v->id() << " ";
    }
    e->write(os);
    os << endl;
    return os.good();
  }
  return false;
}

void OptimizableGraph::clearParameters()
{
  _parameters.clear();
}

bool OptimizableGraph::verifyInformationMatrices(bool verbose) const
{
  bool allEdgeOk = true;
  SelfAdjointEigenSolver<MatrixXd> eigenSolver;
  for (OptimizableGraph::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    Eigen::MatrixXd::MapType information(e->informationData(), e->dimension(), e->dimension());
    // test on symmetry
    bool isSymmetric = information.transpose() == information;
    bool okay = isSymmetric;
    if (isSymmetric) {
      // compute the eigenvalues
      eigenSolver.compute(information, Eigen::EigenvaluesOnly);
      bool isSPD = eigenSolver.eigenvalues()(0) >= 0.;
      okay = okay && isSPD;
    }
    allEdgeOk = allEdgeOk && okay;
    if (! okay) {
      if (verbose) {
        if (! isSymmetric)
          cerr << "Information Matrix for an edge is not symmetric:";
        else
          cerr << "Information Matrix for an edge is not SPD:";
        for (size_t i = 0; i < e->vertices().size(); ++i)
          cerr << " " << e->vertex(i)->id();
        if (isSymmetric)
          cerr << "\teigenvalues: " << eigenSolver.eigenvalues().transpose();
        cerr << endl;
      }
    }
  }
  return allEdgeOk;
}

bool OptimizableGraph::initMultiThreading()
{
# if (defined G2O_OPENMP) && EIGEN_VERSION_AT_LEAST(3,1,0)
  Eigen::initParallel();
# endif
  return true;
}

} // end namespace


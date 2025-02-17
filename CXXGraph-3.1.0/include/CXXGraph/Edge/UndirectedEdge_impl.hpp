/***********************************************************/
/***      ______  ____  ______                 _         ***/
/***     / ___\ \/ /\ \/ / ___|_ __ __ _ _ __ | |__	     ***/
/***    | |    \  /  \  / |  _| '__/ _` | '_ \| '_ \	 ***/
/***    | |___ /  \  /  \ |_| | | | (_| | |_) | | | |    ***/
/***     \____/_/\_\/_/\_\____|_|  \__,_| .__/|_| |_|    ***/
/***                                    |_|			     ***/
/***********************************************************/
/***     Header-Only C++ Library for Graph			     ***/
/***	 Representation and Algorithms				     ***/
/***********************************************************/
/***     Author: ZigRazor ***/
/***	 E-Mail: zigrazor@gmail.com 				     ***/
/***********************************************************/
/***	 Collaboration: ----------- 				     ***/
/***********************************************************/
/***	 License: AGPL v3.0 ***/
/***********************************************************/

#ifndef __CXXGRAPH_UNDIRECTEDEDGE_IMPL_H__
#define __CXXGRAPH_UNDIRECTEDEDGE_IMPL_H__

#pragma once

#include "UndirectedEdge_decl.h"

namespace CXXGraph {

using std::make_unique;
using std::make_shared;

template <typename T>
UndirectedEdge<T>::UndirectedEdge(const CXXGraph::id_t id, const Node<T> &node1,
                                  const Node<T> &node2)
    : Edge<T>(id, node1, node2) {}

template <typename T>
UndirectedEdge<T>::UndirectedEdge(const CXXGraph::id_t id, shared<const Node<T>> node1,
                                  shared<const Node<T>> node2)
    : Edge<T>(id, node1, node2) {}

template <typename T>
UndirectedEdge<T>::UndirectedEdge(
    const CXXGraph::id_t id,
    const std::pair<const Node<T> *, const Node<T> *> &nodepair)
    : Edge<T>(id, nodepair) {}

template <typename T>
UndirectedEdge<T>::UndirectedEdge(
    const CXXGraph::id_t id,
    const std::pair<shared<const Node<T>>, shared<const Node<T>>> &nodepair)
    : Edge<T>(id, nodepair) {}

template <typename T>
UndirectedEdge<T>::UndirectedEdge(const Edge<T> &edge)
    : UndirectedEdge(edge.getId(), *(edge.getNodePair().first),
                     *(edge.getNodePair().second)) {}

template <typename T>
const Node<T> &UndirectedEdge<T>::getNode1() const {
  return *(Edge<T>::getNodePair().first);
}

template <typename T>
const Node<T> &UndirectedEdge<T>::getNode2() const {
  return *(Edge<T>::getNodePair().second);
}

template <typename T>
const std::optional<bool> UndirectedEdge<T>::isDirected() const {
  return false;
}

template <typename T>
const std::optional<bool> UndirectedEdge<T>::isWeighted() const {
  return false;
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const UndirectedEdge<T> &edge) {
  os << "((Node: " << edge.getNode1().getId() << ")) <----- |Edge: #"
     << edge.getId() << "|-----> ((Node: " << edge.getNode2().getId() << "))";
  return os;
}
}  // namespace CXXGraph

#endif  // __CXXGRAPH_UNDIRECTEDEDGE_IMPL_H__

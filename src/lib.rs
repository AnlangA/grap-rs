#![no_std]
extern crate alloc;
extern crate core;

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use alloc::vec;
use log::info;

pub type NodeId = usize;
pub type EdgeId = usize;
pub type UnitId = usize;

#[derive(Clone, Debug)]
pub struct Node<N> {
    pub data: N,
}

#[derive(Clone, Debug)]
pub struct Edge<E> {
    pub a: NodeId,
    pub b: NodeId,
    pub data: E,
}

#[derive(Clone, Debug)]
pub struct Unit<U> {
    pub data: U,
}

/// A static, undirected graph with contiguous IDs (0..n) for nodes, edges, and units.
/// No deletion APIs are provided by design.
#[derive(Debug)]
pub struct Graph<N, U, E> {
    nodes: Vec<Node<N>>,
    edges: Vec<Edge<E>>,
    units: Vec<Unit<U>>,
    adjacency: Vec<Vec<(NodeId, EdgeId)>>,
    node_units: Vec<Vec<UnitId>>,
    unit_to_node: Vec<Option<NodeId>>,
}

/// Error types for graph operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GraphError {
    /// Node ID is out of range.
    InvalidNodeId(NodeId),
    /// Edge ID is out of range.
    InvalidEdgeId(EdgeId),
    /// Unit ID is out of range.
    InvalidUnitId(UnitId),
    /// Attempted to create an edge between the same node.
    SelfLoopNotAllowed,
    /// Attempted to create a duplicate edge.
    DuplicateEdge,
}

impl<N, U, E> Graph<N, U, E> {
    /// Create an empty graph with the given capacities.
    pub fn with_capacities(node_cap: usize, unit_cap: usize, edge_cap: usize) -> Self {
        Graph {
            nodes: Vec::with_capacity(node_cap),
            edges: Vec::with_capacity(edge_cap),
            units: Vec::with_capacity(unit_cap),
            adjacency: Vec::with_capacity(node_cap),
            node_units: Vec::with_capacity(node_cap),
            unit_to_node: Vec::with_capacity(unit_cap),
        }
    }

    /// Create an empty graph with zero capacities.
    #[inline]
    pub fn new() -> Self {
        Self::with_capacities(0, 0, 0)
    }

    /// Number of nodes
    #[inline]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of edges
    #[inline]
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Number of units
    #[inline]
    pub fn unit_count(&self) -> usize {
        self.units.len()
    }

    /// Add a node and return its id (contiguous starting from 0)
    pub fn add_node(&mut self, data: N) -> NodeId {
        let id = self.nodes.len();
        self.nodes.push(Node { data });
        self.adjacency.push(Vec::new());
        self.node_units.push(Vec::new());
        id
    }

    /// Add an undirected edge between nodes `a` and `b` and return its id.
    pub fn add_edge(&mut self, a: NodeId, b: NodeId, data: E) -> Result<EdgeId, GraphError> {
        if a >= self.nodes.len() {
            return Err(GraphError::InvalidNodeId(a));
        }
        if b >= self.nodes.len() {
            return Err(GraphError::InvalidNodeId(b));
        }
        if a == b {
            return Err(GraphError::SelfLoopNotAllowed);
        }
        
        // Check if edge already exists
        if self.are_connected(a, b) {
            return Err(GraphError::DuplicateEdge);
        }
        
        let id = self.edges.len();
        self.edges.push(Edge { a, b, data });
        self.adjacency[a].push((b, id));
        self.adjacency[b].push((a, id));

        Ok(id)
    }

    /// Add a unit on a node and return its id.
    pub fn add_unit_on_node(&mut self, node: NodeId, data: U) -> Result<UnitId, GraphError> {
        if node >= self.nodes.len() {
            return Err(GraphError::InvalidNodeId(node));
        }
        let id = self.units.len();
        self.units.push(Unit { data });
        self.node_units[node].push(id);
        // ensure unit_to_node has contiguous mapping
        self.unit_to_node.push(Some(node));

        Ok(id)
    }

    /// Get immutable reference to a node by id.
    pub fn node(&self, id: NodeId) -> Option<&Node<N>> {
        self.nodes.get(id)
    }
    /// Get mutable reference to a node by id.
    pub fn node_mut(&mut self, id: NodeId) -> Option<&mut Node<N>> {
        self.nodes.get_mut(id)
    }
    /// Get immutable reference to an edge by id.
    pub fn edge(&self, id: EdgeId) -> Option<&Edge<E>> {
        self.edges.get(id)
    }
    /// Get immutable reference to a unit by id.
    pub fn unit(&self, id: UnitId) -> Option<&Unit<U>> {
        self.units.get(id)
    }
    /// Get mutable reference to a unit by id.
    pub fn unit_mut(&mut self, id: UnitId) -> Option<&mut Unit<U>> {
        self.units.get_mut(id)
    }
    /// Get mutable reference to an edge by id.
    pub fn edge_mut(&mut self, id: EdgeId) -> Option<&mut Edge<E>> {
        self.edges.get_mut(id)
    }

    /// Get the node id where a given unit resides.
    pub fn node_of_unit(&self, unit: UnitId) -> Option<NodeId> {
        self.unit_to_node.get(unit).and_then(|opt| *opt)
    }

    /// Get neighbors of a node as slice of (neighbor_node_id, edge_id).
    pub fn neighbors(&self, node: NodeId) -> Option<&[(NodeId, EdgeId)]> {
        self.adjacency.get(node).map(|v| v.as_slice())
    }

    /// Get units located on a node as slice of unit ids.
    pub fn units_on_node(&self, node: NodeId) -> Option<&[UnitId]> {
        self.node_units.get(node).map(|v| v.as_slice())
    }

    /// Iterate neighbor node ids for a given node.
    pub fn neighbor_nodes(&self, node: NodeId) -> Option<impl Iterator<Item = NodeId> + '_> {
        self.adjacency.get(node).map(|v| v.iter().map(|(n, _e)| *n))
    }

    /// Iterate neighbor edges for a given node.
    pub fn neighbor_edges(&self, node: NodeId) -> Option<impl Iterator<Item = EdgeId> + '_> {
        self.adjacency.get(node).map(|v| v.iter().map(|(_n, e)| *e))
    }

    /// Get all nodes as an iterator of (node_id, node_data).
    pub fn nodes(&self) -> impl Iterator<Item = (NodeId, &Node<N>)> {
        self.nodes.iter().enumerate()
    }

    /// Get all edges as an iterator of (edge_id, edge_data).
    pub fn edges(&self) -> impl Iterator<Item = (EdgeId, &Edge<E>)> {
        self.edges.iter().enumerate()
    }

    /// Get all units as an iterator of (unit_id, unit_data).
    pub fn units(&self) -> impl Iterator<Item = (UnitId, &Unit<U>)> {
        self.units.iter().enumerate()
    }

    /// Check if two nodes are connected by an edge.
    pub fn are_connected(&self, a: NodeId, b: NodeId) -> bool {
        if a >= self.nodes.len() || b >= self.nodes.len() {
            return false;
        }
        self.adjacency[a].iter().any(|(n, _e)| *n == b)
    }

    /// Get the edge between two nodes if it exists.
    pub fn edge_between(&self, a: NodeId, b: NodeId) -> Option<EdgeId> {
        if a >= self.nodes.len() || b >= self.nodes.len() {
            return None;
        }
        self.adjacency[a].iter().find(|(n, _e)| *n == b).map(|(_n, e)| *e)
    }

    /// Get the degree of a node (number of connected edges).
    pub fn degree(&self, node: NodeId) -> Option<usize> {
        self.adjacency.get(node).map(|v| v.len())
    }

    /// Print adjacency list with one neighbor per line:
    /// nID: nbr(eid)
    /// If a node has no neighbors, prints nID: -
    pub fn log_adjacency(&self) {
        info!("adjacency list:");
        for nid in 0..self.nodes.len() {
            if let Some(neigh) = self.adjacency.get(nid) {
                if neigh.is_empty() {
                    info!("{:>3}: -", nid);
                } else {
                    for (nbr, eid) in neigh.iter() {
                        info!("{:>3}: {}({})", nid, nbr, eid);
                    }
                }
            } else {
                info!("n{:>3}: -", nid);
            }
        }
    }

    /// Find all nodes with no edges (isolated nodes).
    pub fn isolated_nodes(&self) -> impl Iterator<Item = NodeId> + '_ {
        self.adjacency.iter()
            .enumerate()
            .filter(|(_, neighbors)| neighbors.is_empty())
            .map(|(id, _)| id)
    }

    /// Find all connected components in the graph using DFS.
    /// This function returns a Vec of all connected components.
    pub fn connected_components(&self) -> Vec<Vec<NodeId>> {
        let mut visited = vec![false; self.nodes.len()];
        let mut components = Vec::new();
        
        for node_id in 0..self.nodes.len() {
            if !visited[node_id] {
                let mut component = Vec::new();
                self.dfs(node_id, &mut visited, &mut component);
                components.push(component);
            }
        }
        
        components
    }
    
    /// Depth-first search helper function for connected_components.
    fn dfs(&self, node: NodeId, visited: &mut [bool], component: &mut Vec<NodeId>) {
        visited[node] = true;
        component.push(node);
        
        if let Some(neighbors) = self.neighbors(node) {
            for (neighbor, _) in neighbors {
                if !visited[*neighbor] {
                    self.dfs(*neighbor, visited, component);
                }
            }
        }
    }

    /// Calculate the shortest path between two nodes using BFS (unweighted graph).
    /// Returns None if no path exists.
    pub fn shortest_path(&self, start: NodeId, end: NodeId) -> Option<Vec<NodeId>> {
        if start >= self.nodes.len() || end >= self.nodes.len() {
            return None;
        }
        
        if start == end {
            return Some(vec![start]);
        }
        
        let mut visited = vec![false; self.nodes.len()];
        let mut queue = VecDeque::new();
        let mut prev = vec![None; self.nodes.len()];
        
        queue.push_back(start);
        visited[start] = true;
        
        while let Some(current) = queue.pop_front() {
            if let Some(neighbors) = self.neighbors(current) {
                for (neighbor, _) in neighbors {
                    if *neighbor == end {
                        // Reconstruct path
                        let mut path = vec![end];
                        let mut node = current;
                        
                        while let Some(p) = prev[node] {
                            path.push(p);
                            node = p;
                        }
                        
                        path.reverse();
                        path.push(end);
                        return Some(path);
                    }
                    
                    if !visited[*neighbor] {
                        visited[*neighbor] = true;
                        prev[*neighbor] = Some(current);
                        queue.push_back(*neighbor);
                    }
                }
            }
        }
        
        None
    }

    /// Get all units across all nodes as an iterator of (unit_id, node_id, unit_data).
    pub fn all_units(&self) -> impl Iterator<Item = (UnitId, NodeId, &Unit<U>)> + '_ {
        self.units.iter().enumerate().filter_map(|(unit_id, unit)| {
            self.node_of_unit(unit_id).map(|node_id| (unit_id, node_id, unit))
        })
    }
}

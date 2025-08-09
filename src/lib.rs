#![no_std]
extern crate alloc;

use alloc::vec::Vec;
use core::marker::PhantomData;
use log::{error, info};

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
    // id == index in these vectors
    nodes: Vec<Node<N>>,
    edges: Vec<Edge<E>>,                 // edge id == index
    units: Vec<Unit<U>>,                 // unit id == index
    adjacency: Vec<Vec<(NodeId, EdgeId)>>, // per-node neighbors with edge id
    node_units: Vec<Vec<UnitId>>,        // per-node unit ids
    unit_to_node: Vec<Option<NodeId>>,   // unit id -> node id
    _phantom: PhantomData<(N, U, E)>,
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
            _phantom: PhantomData,
        }
    }

    /// Create an empty graph with zero capacities.
    #[inline]
    pub fn new() -> Self { Self::with_capacities(0, 0, 0) }

    /// Number of nodes
    #[inline]
    pub fn node_count(&self) -> usize { self.nodes.len() }
    /// Number of edges
    #[inline]
    pub fn edge_count(&self) -> usize { self.edges.len() }
    /// Number of units
    #[inline]
    pub fn unit_count(&self) -> usize { self.units.len() }

    /// Add a node and return its id (contiguous starting from 0)
    pub fn add_node(&mut self, data: N) -> NodeId {
        let id = self.nodes.len();
        self.nodes.push(Node { data });
        self.adjacency.push(Vec::new());
        self.node_units.push(Vec::new());
        id
    }

    /// Add an undirected edge between nodes `a` and `b` and return its id.
    pub fn add_edge(&mut self, a: NodeId, b: NodeId, data: E) -> Option<EdgeId> {
        if a >= self.nodes.len() || b >= self.nodes.len() {
            error!("graph: add_edge invalid nodes a={}, b={} (node_count={})", a, b, self.nodes.len());
            return None;
        }
        let id = self.edges.len();
        self.edges.push(Edge { a, b, data });
        self.adjacency[a].push((b, id));
        self.adjacency[b].push((a, id));

        Some(id)
    }

    /// Add a unit on a node and return its id.
    pub fn add_unit_on_node(&mut self, node: NodeId, data: U) -> Option<UnitId> {
        if node >= self.nodes.len() {
            error!("graph: add_unit_on_node invalid node={} (node_count={})", node, self.nodes.len());
            return None;
        }
        let id = self.units.len();
        self.units.push(Unit { data });
        self.node_units[node].push(id);
        // ensure unit_to_node has contiguous mapping
        self.unit_to_node.push(Some(node));

        Some(id)
    }

    /// Get immutable reference to a node by id.
    pub fn node(&self, id: NodeId) -> Option<&Node<N>> { self.nodes.get(id) }
    /// Get mutable reference to a node by id.
    pub fn node_mut(&mut self, id: NodeId) -> Option<&mut Node<N>> { self.nodes.get_mut(id) }
    /// Get immutable reference to an edge by id.
    pub fn edge(&self, id: EdgeId) -> Option<&Edge<E>> { self.edges.get(id) }
    /// Get immutable reference to a unit by id.
    pub fn unit(&self, id: UnitId) -> Option<&Unit<U>> { self.units.get(id) }
    /// Get mutable reference to a unit by id.
    pub fn unit_mut(&mut self, id: UnitId) -> Option<&mut Unit<U>> { self.units.get_mut(id) }

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
}

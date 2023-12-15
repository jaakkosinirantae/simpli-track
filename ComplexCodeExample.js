/*
 * Filename: ComplexCodeExample.js
 * Description: This code implements a complex algorithm to find the shortest path between multiple points in a graph using the A* search algorithm.
 */

class Node {
  constructor(value, heuristic) {
    this.value = value;
    this.heuristic = heuristic;
    this.edges = [];
    this.parent = null;
    this.gScore = Infinity;
    this.fScore = Infinity;
  }

  addEdge(neighbor, weight) {
    this.edges.push({ node: neighbor, weight });
  }

  getNeighbors() {
    return this.edges.map(edge => edge.node);
  }
}

class Graph {
  constructor() {
    this.nodes = [];
  }

  addNode(value, heuristic) {
    this.nodes.push(new Node(value, heuristic));
  }

  getNode(value) {
    return this.nodes.find(node => node.value === value);
  }

  addEdge(source, target, weight) {
    const sourceNode = this.getNode(source);
    const targetNode = this.getNode(target);

    if (!sourceNode || !targetNode) {
      throw new Error('Invalid node value');
    }

    sourceNode.addEdge(targetNode, weight);
    targetNode.addEdge(sourceNode, weight);
  }

  heuristic(start, end) {
    // Returns an estimated distance between two nodes
    // Implement your own heuristic function here
    return Math.abs(start - end);
  }

  reconstructPath(current) {
    const path = [];
    while (current.parent) {
      path.push(current.value);
      current = current.parent;
    }
    path.push(current.value);
    return path.reverse();
  }

  aStar(startValue, endValue) {
    const startNode = this.getNode(startValue);
    const endNode = this.getNode(endValue);

    if (!startNode || !endNode) {
      throw new Error('Invalid node value');
    }

    const openSet = [startNode];
    const closedSet = [];

    startNode.gScore = 0;
    startNode.fScore = this.heuristic(startNode.value, endNode.value);

    while (openSet.length) {
      let currentNode = openSet[0];

      if (currentNode === endNode) {
        return this.reconstructPath(currentNode);
      }

      openSet.splice(0, 1);
      closedSet.push(currentNode);

      const neighbors = currentNode.getNeighbors();

      for (let i = 0; i < neighbors.length; i++) {
        const neighbor = neighbors[i];
        if (closedSet.includes(neighbor)) continue;

        const tentativeGScore = currentNode.gScore + 1;

        if (!openSet.includes(neighbor)) {
          openSet.push(neighbor);
        } else if (tentativeGScore >= neighbor.gScore) {
          continue;
        }

        neighbor.parent = currentNode;
        neighbor.gScore = tentativeGScore;
        neighbor.fScore = neighbor.gScore + this.heuristic(
          neighbor.value,
          endNode.value
        );

        openSet.sort((a, b) => a.fScore - b.fScore);
      }
    }

    return [];
  }
}

// Usage example
const graph = new Graph();

graph.addNode('A', 3);
graph.addNode('B', 2);
graph.addNode('C', 1);
graph.addNode('D', 5);
graph.addNode('E', 4);
graph.addNode('F', 2);
graph.addNode('G', 6);
graph.addNode('H', 3);

graph.addEdge('A', 'B', 1);
graph.addEdge('A', 'C', 2);
graph.addEdge('C', 'D', 3);
graph.addEdge('C', 'E', 2);
graph.addEdge('E', 'F', 1);
graph.addEdge('D', 'F', 3);
graph.addEdge('D', 'G', 4);
graph.addEdge('F', 'H', 2);
graph.addEdge('G', 'H', 1);

const shortestPath = graph.aStar('A', 'H');
console.log(shortestPath);

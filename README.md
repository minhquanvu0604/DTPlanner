[User Manual](https://doc.cgal.org/latest/Triangulation_2/index.html#Section_2D_Triangulations_Flexibility)

2.1 The Set of Faces

The single unbounded face of this partition is the complementary of the convex hull. [...]
Therefore, a fictitious vertex, called the **infinite vertex** is added to the triangulation as well as **infinite edges\*** and infinite faces incident to it.

2.2 A Representation Based on Faces and Vertices

3 Software Design

4 Basic Triangulations

4.1 Description
The class Triangulation_2<Traits,Tds> serves as a base class for the other 2D triangulations classes and implements the user interface to a triangulation.

The vertices and faces of the triangulations are accessed through handles, iterators and circulators. A handle is a model of the concept Handle which basically offers the two dereference operators * and ->. A circulator is a type devoted to visit circular sequences. Handles are used whenever the accessed element is not part of a sequence. Iterators and circulators are used to visit all or parts of the triangulation.

The iterators and circulators are all bidirectional and non mutable. The circulators and iterators are convertible to the handles with the same value type, so that when calling a member function, any handle type argument can be replaced by an iterator or a circulator with the same value type.




[USAGE GUIDE]

In CGAL's Delaunay Triangulation data structure, and in many computational geometry libraries, the primary focus is on efficient computation of the triangulation and queries related to it, such as nearest neighbor searches or point location queries. The storage mechanism is optimized for these operations rather than direct retrieval of an edge given its vertices.

This design choice is guided by the typical use cases of such libraries, where direct edge retrieval by vertices is less common than other operations. Additionally, the internal representation of triangulations often involves complex data structures like half-edge data structures, which are optimized for operations like traversing and modifying the triangulation rather than direct access to a specific edge.

In short, while it's technically possible to provide a direct edge retrieval function, it's not a common requirement in the design and use of these libraries, and doing so might not align with the primary optimization goals of the data structure.



Traversing the adjacency in CGAL's Delaunay Triangulation effectively often involves using circulators and iterators provided by CGAL. These tools are designed to navigate through the vertices, edges, and faces of the triangulation. Here are some common ways to traverse the triangulation


NOTE - EDGE:
---nearest_edge.first: This is a Face_handle pointing to one of the faces (triangle) adjacent to the edge. 
---nearest_edge.second: This is an integer (0, 1, or 2) that specifies which edge of the triangle (nearest_edge.first) we are referring to. It helps to identify the exact edge within the triangle.
This is due to the fact that the triangulation is built upon topological relations, not just geometric ones.

# Hospital Navigation for Robots

## Overview
This project delves into improving robots' navigation within hospital environments, focusing on the efficiency and reliability of different pathfinding methodologies. Specifically, it evaluates the Probabilistic Roadmap Method (PRM) against well-known Dijkstra and A* algorithms to identify the most effective strategy for guiding robots through the complex and dynamic spaces of hospitals. The objective is to optimize robots' ability to navigate, avoid obstacles, and deliver items swiftly, enhancing the operational flow of hospital services and ensuring timely patient care.

The exploration is grounded in the context of Hadassah Hospital's Emergency Room (ER) in Jerusalem, providing a realistic backdrop for our simulations. This choice allows for a detailed examination of navigation challenges within a particularly critical and complex hospital area.

## Problem Description
Navigating a hospital environment poses significant challenges for robots due to the presence of both static and dynamic obstacles, and the need to adapt to changes within the hospital layout swiftly. The project aims to devise a system capable of dynamically selecting the optimal route for a robot moving from one point to another, enhancing efficiency in hospital logistics.

## Contributing
We welcome contributions! Please read our contributing guidelines to learn how you can contribute to improving hospital robot navigation. Whether it's adding new features, improving documentation, or reporting bugs, your help is appreciated.

1. Fork the project
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a pull request

# Hospital Navigation for Robots

## Data Utilized
The project leverages detailed layouts of Hadassah Hospital, specifically its ER, along with movement patterns of obstacles, to assess the effectiveness of various navigation methods under real-world conditions. This practical approach ensures that the findings are directly applicable to improving hospital operations.

## Algorithms
### PRM Algorithm
The PRM algorithm generates a network of possible routes through the hospital's open spaces, linking points with paths that bypass obstacles. It then searches this network to find the most efficient and safe route, using the detailed layout of Hadassah Hospital's ER as a practical example.

### Dijkstra Algorithm
This algorithm takes a methodical approach, creating a graph based on the hospital's layout where nodes represent locations, and edges represent connections like corridors. It systematically explores all possible routes from the starting point, continually updating the shortest path to each node until it identifies the most efficient route.

### A* Algorithm
A* combines the actual distance traveled with an estimated distance to the goal, prioritizing paths that appear to lead more directly towards the destination. This heuristic approach allows A* to find efficient paths more quickly than algorithms that only consider the actual distance traveled, making it particularly suitable for dynamic environments like hospitals.

## Hadassah ER Hospital
The research is specifically tailored around the intricacies of navigating the Emergency Room at Hadassah Hospital in Jerusalem. The simulation environment meticulously replicates the ER's layout, translating various hospital features into graph nodes and obstacles. This specificity ensures that the study's outcomes are grounded in real-world applicability, offering insights that could directly benefit hospital logistics and patient care within Hadassah Hospital and potentially be adapted to other healthcare facilities.

## Results and Future Directions
Our findings indicate a nuanced performance landscape where the A* algorithm generally outperforms Dijkstra's method in terms of efficiency and speed, particularly with increasing numbers of nodes (or complexity). However, Dijkstra's algorithm showcases robustness, consistently finding the shortest path, albeit at a slower pace. This research lays the groundwork for further exploration into dynamic obstacle avoidance and the integration of machine learning techniques to adapt pathfinding algorithms in real-time, promising enhancements to both algorithmic efficiency and practical utility in hospital settings.

The project's emphasis on Hadassah Hospital's ER not only provides a detailed framework for our simulations but also underscores the potential of advanced pathfinding algorithms to address real-world challenges in healthcare logistics and patient care delivery.

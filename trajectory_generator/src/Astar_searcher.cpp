#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

bool tie_break = false;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
//                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only TODO: careful
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear(); // Note: the pointers in this set copy pointers to GridNodeMap
    edgeCostSets.clear();
    
    
    // idea index -> coordinate -> edgecost
    if(currentPtr == nullptr)
        std::cout << "Error: Current pointer is null!" << endl;


    Eigen::Vector3i thisNode = currentPtr -> index;
    int this_x = thisNode[0];
    int this_y = thisNode[1];
    int this_z = thisNode[2];
    auto this_coord = currentPtr -> coord;
    int  n_x, n_y, n_z;
    double dist;
    GridNodePtr temp_ptr = nullptr;
    Eigen::Vector3d n_coord;

    for(int i = -1;i <= 1;++i ){
        for(int j = -1;j <= 1;++j ){
            for(int k = -1;k <= 1;++k){
                if( i == 0 && j == 0 && k == 0)
                    continue; // to avoid this node
                n_x = this_x + i;
                n_y = this_y + j;
                n_z = this_z + k;

                if( (n_x < 0) || (n_x > (GLX_SIZE - 1)) || (n_y < 0) || (n_y > (GLY_SIZE - 1) ) || (n_z < 0) || (n_z > (GLZ_SIZE - 1)))
                    continue; // to avoid index problem
                if(isOccupied(n_x, n_y, n_z))
                    continue; // to avoid obstacles
                // put the pointer into neighborPtrSets
                temp_ptr = GridNodeMap[n_x][n_y][n_z];

                if(temp_ptr->id == -1) continue; // todo to check this; why the node can transversing the obstacles

                n_coord = temp_ptr->coord;


                if(temp_ptr == currentPtr){
                    std::cout << "Error: temp_ptr == currentPtr)" << std::endl;
                }

                if( (std::abs(n_coord[0] - this_coord[0]) < 1e-6) and (std::abs(n_coord[1] - this_coord[1]) < 1e-6) and (std::abs(n_coord[2] - this_coord[2]) < 1e-6 )){
                    std::cout << "Error: Not expanding correctly!" << std::endl;
                    std::cout << "n_coord:" << n_coord[0] << " "<<n_coord[1]<<" "<<n_coord[2] << std::endl;
                    std::cout << "this_coord:" << this_coord[0] << " "<<this_coord[1]<<" "<<this_coord[2] << std::endl;

                    std::cout << "current node index:" << this_x << " "<< this_y<<" "<< this_z << std::endl;
                    std::cout << "neighbor node index:" << n_x << " "<< n_y<<" "<< n_z << std::endl;
                }

                dist = std::sqrt( (n_coord[0] - this_coord[0]) * (n_coord[0] - this_coord[0])+
                        (n_coord[1] - this_coord[1]) * (n_coord[1] - this_coord[1])+
                        (n_coord[2] - this_coord[2]) * (n_coord[2] - this_coord[2]));
                neighborPtrSets.push_back(temp_ptr); // calculate the cost in edgeCostSets: inf means that is not unexpanded
                edgeCostSets.push_back(dist); // put the cost inot edgeCostSets

            }
        }
    }



}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{

    double h;
    auto node1_coord = node1->coord;
    auto node2_coord = node2->coord;

    // Heuristics 1: Manhattan
   h = std::abs(node1_coord(0) - node2_coord(0) ) +
           std::abs(node1_coord(1) - node2_coord(1) ) +
           std::abs(node1_coord(2) - node2_coord(2) );

    // Heuristics 2: Euclidean
//    h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
//            std::pow((node1_coord(1) - node2_coord(1)), 2 ) +
//            std::pow((node1_coord(2) - node2_coord(2)), 2 ));

    // Heuristics 3: Diagnol distance
        // double dx = std::abs(node1_coord(0) - node2_coord(0) );
        // double dy = std::abs(node1_coord(1) - node2_coord(1) );
        // double dz = std::abs(node1_coord(2) - node2_coord(2) );
        // double min_xyz = std::min({dx, dy, dz});
        // h = dx + dy + dz + (std::sqrt(3.0) -3) * min_xyz; // idea: diagnol is a short-cut, find out how many short-cuts can be realized

        if(tie_break){
            double p = 1.0 / 1000;
            h *= (1.0 + p);

        }
//    std::cout <<"h is: "<< h << std::endl;
    return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); // todo Note: modified, insert the pointer GridNodeMap[i][j][k] to the start node in grid map

    // three dimension pointer GridNodeMap[i][j][k] is pointed to a struct GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord);
    // assign g(xs) = 0, g(n) = inf (already done in initialzation of struct)
    // mark start point as visited(expanded) (id 0: no operation, id: 1 in OPEN, id -1: in CLOSE )

    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] -> id = 1;


    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    Eigen::Vector3i current_idx; // record the current index

    // this is the main loop
    while ( !openSet.empty() ){
    

        currentPtr = openSet.begin() -> second; // first T1, second T2
        openSet.erase(openSet.begin()); // remove the node with minimal f value
        current_idx = currentPtr->index;
        GridNodeMap[current_idx[0]][current_idx[1]][current_idx[2]] -> id = -1;// update the id in grid node map
        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

 
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            // double collision_cost  = calCollisionCost(neighborPtr);
            double collision_cost = 0.0;
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set


                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr) + collision_cost;
                neighborPtr->cameFrom = currentPtr; // todo shallow copy or deep copy
                // push node "m" into OPEN
                openSet.insert(make_pair(neighborPtr -> fScore, neighborPtr));
                neighborPtr -> id = 1;
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
     
                // shall update: gScore; fScore; cameFrom, mayby direction
                if(neighborPtr -> gScore > (currentPtr -> gScore + edgeCostSets[i])){
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr,endPtr) + collision_cost;
                    neighborPtr -> cameFrom = currentPtr;
                }

                continue;
            }
            else{//this node is in closed set
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    auto ptr = terminatePtr;
    while(ptr -> cameFrom != NULL){
        gridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}

// if the difference of f is trivial, then choose then prefer the path along the straight line from start to goal

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.rows();
  for (int dim = 0; dim < 3; dim++) {
    // VectorXd coeff = polyCoeff.block(0, dim, _poly_num1D, 1);
    VectorXd coeff = polyCoeff.col(dim);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  
  int seg_num = int(time.size());
  for(int i = 0; i < seg_num; ++i){
    double ts = time(i);
    Eigen::MatrixXd coeff = polyCoeff.block(6 * i, 0, 6, 3);
    for(double t = 0.0; t < ts; t += 0.15){
      Vector3d ps = getPosPoly(coeff, i, t);
      Vector3i index = coord2gridIndex(ps);
      if(isOccupied(index)) {
        ROS_WARN("Collision will happend at %drd segment!", i);
         return i;
      }
    }
  }
    ROS_WARN("the Trajectory is safe!");
  return unsafe_segment;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath;
  vector<Vector3d> preSubPath;
  vector<Vector3d>  postSubPath; 
  /**
   *   implement the RDP algorithm
   * **/
  if(int(path.size()) <= 2) return path;

  double d_max = 0.0;
  int index = 0;
  int end_idx = int(path.size()) - 1;
  for(int i  = 1; i < end_idx ; ++i){
    Vector3d coord1  = path[end_idx] - path[0];
    Vector3d coord2 = path[i] - path[0];
    Vector3d d_vector = coord1.cross(coord2);
    double d = d_vector.norm() / coord1.norm();
    if(d > d_max){
      index = i;
      d_max = d;
    }
  }

  if(d_max > path_resolution){
    vector<Vector3d>::const_iterator first1 = path.begin();
    vector<Vector3d>::const_iterator end1 = path.begin() + index + 1;

    vector<Vector3d>::const_iterator first2 = path.begin() + index ;
    vector<Vector3d>::const_iterator end2 = path.end();

    vector<Vector3d> prePath(first1, end1);
    vector<Vector3d> postPath(first2, end2);

    preSubPath = pathSimplify(prePath, path_resolution);
    postSubPath = pathSimplify(postPath, path_resolution);

    subPath.insert(subPath.end(), preSubPath.begin(), preSubPath.end() - 1);
    subPath.insert(subPath.end(), postSubPath.begin(), postSubPath.end());
  }else{
    subPath.push_back(path[0]);
    subPath.push_back(path[end_idx]);
  }
  return subPath;
}

double AstarPathFinder::calCollisionCost(GridNodePtr gridnode) {
    Eigen::Vector3i index = gridnode->index;
    double cost = 0.0;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                if (i == 0 && j ==0 && k ==0) continue;
                Vector3i neigbors_index(index[0] + i, index[1] + j, index[2] + k);
                if (isOccupied(neigbors_index)) {
                    cost = 50.0;
                    break;
                }
            }
        }
    }
    return  cost;
}
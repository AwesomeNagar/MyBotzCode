#include <iostream>
#include <algorithm>
#include <map>
#include <queue>
#include <vector>
#include <unordered_set>
#include <cstring>
#include<math.h>
#include<cstdlib>
using namespace std;
struct point {
    float x,y;
};
struct edge{
    int p1,p2;
    double len;
};
int compare (const void *a, const void *b)
{

    if (((edge*)a)->len < ((edge*)b)->len) return -1;
    return 1;
};
typedef map<int, queue<int>*> mup;
//Ord will store the order in which we go to each vertex
vector<int> *ord = NULL;
float mindist = 100000;
int last = 0;


int findParent(int* parent, int i){
        //Given a vertex i, find its parent
        if (parent[i] == -1){
            return i;
        }else{
            return findParent(parent,parent[i]);
        }
   }


mup* treeMake(edge* sortedge,int sizeedge, int n){
    mup* rettree = new mup;
    int* parent = new int[n+1];
    //Everyone is currently their own parent
    memset(parent,0xff,(n+1)*sizeof(int));
    for (int i = 0; i < sizeedge; i++){
        edge* next = &sortedge[i];
        int p1 = findParent(parent, next->p1);
        int p2 = findParent(parent, next->p2);
        //If their parents are the same, we would form a cycle, so don't continue if they are
        if (p1 != p2) {
            //Put them in the same subset
            parent[p1] = p2;

            //Add the paths TWICE so our double tree algorithm is valid
            if(rettree->count(next->p1) == 0){
                (*rettree)[next->p1] = new queue<int>();//TODO
            }
            (*rettree)[next->p1]->push(next->p2);
            if(rettree->count(next->p2) == 0){
                (*rettree)[next->p2] = new queue<int>();//TODO
            }
            (*rettree)[next->p2]->push(next->p1);
        }
    }
    delete parent;
    return rettree;
}
void DFS(mup& tree, int focus, vector<int>& order, unordered_set<int>& visited, float** edgelen, float dist){
    if (tree.empty()){
        //Return condition is if we have visited every vertex
        if (mindist > dist) {
            if (!ord)
            {
                delete ord;
            }
            ord = new vector<int>(order);
            mindist = dist;
        }
        return;
    }
    if (tree.count(focus) == 0){
        return;
    }

    //This is to reset our visited array at the end so our DFS works
    bool seen = (visited.count(focus) == 1);
    if (!seen) {
        order.push_back(focus);
    }
    visited.insert(focus);
    mup::iterator it = tree.find(focus);
    queue<int>* qu = (*it).second;
    for (int i = 0; i < qu->size(); i++){
        //Remove the path that we will travel
        int num = qu->front();
        qu->pop();
        if (qu->empty()){
            tree.erase(focus);
        }

        //Find the distance that we will need to travel the path
        if(visited.count(num) == 0){
            dist += edgelen[num][last];
            last = num;
        }
        //Recurse
        DFS(tree,num,order,visited,edgelen,dist);
        if(visited.count(num) == 0){
            last = focus;
            dist -= edgelen[num][last];
        }
        //Reset our tree for the next iteration of the for loop
        if (tree.count(focus) == 0){
            tree.insert( pair<int,queue<int>*>(focus,qu) );
        }
        qu->push(num);

    }
    if (!seen) {
        order.pop_back();
        visited.erase(focus);
    }
}
int main(void) {
    int n;
    cin>>n;
    point** arr = (point**)malloc(sizeof(point *)*(n + 1));
    arr[0] = new point();
    arr[0]->x = -1;
    arr[0]->y = -1;
    //Lengths of edges between two vertices. edgelen[i][j] is the length between the i and j vertices
    float** edgelen = (float**)malloc(sizeof(float*)*(n+1));
    for (int i = 0; i < n+1; ++i)
    {
        edgelen[i] = (float*)malloc(sizeof(float)*(n+1));
    }

    //List of edges that will be sorted by edgelength
    edge* sortedge = (edge*)malloc(sizeof(edge)*(n*(n-1)/2));

    for (int i = 1; i <= n; i++) {
        arr[i] = new point();
        cin>>arr[i]->x;
        cin>>arr[i]->y;
    }
    int ind = 0;
    //Read in the edges
    for (int i = 1; i <= n; i++) {
        for (int j = i + 1; j <= n; j++) {
            edgelen[i][j] = (float)pow((pow(arr[i]->x - arr[j]->x, 2) + pow(arr[i]->y - arr[j]->y, 2)),0.5);
            edgelen[j][i] = edgelen[i][j];
            //sortedge[ind] = new edge();
            sortedge[ind].p1 = i;
            sortedge[ind].p2 = j;
            sortedge[ind].len = edgelen[j][i];
            ind++;
        }
    }
    //Sort by edgelength
    qsort(sortedge, n*(n-1)/2, sizeof(struct edge), compare);

    //Make the Minimum Spanning Tree. tree maps a source vertex to all of its destination vertices
    mup* tree = treeMake(sortedge,n*(n-1)/2,n);

    //Place the dummy point of length 0 to account for cycles. Add it twice for our double tree algo to work
    if (tree->count(0) == 0) {
        //(*tree)[0] = new queue<int>();//TODO
        tree->insert( pair<int,queue<int>*>(0,new queue<int>) );

    }
    mup::iterator it1 = tree->find(0);
    queue<int>* qu1 = (*it1).second;
    qu1->push(1);
    if (tree->count(1) == 0) {
        //(*tree)[1] = new queue<int>();//TODO
        tree->insert( pair<int,queue<int>*>(1,new queue<int>) );
    }
    mup::iterator it2 = tree->find(1);
    queue<int>* qu2 = (*it2).second;
    qu2->push(0);

    //Run DFS on our MST to find the best path possible
    vector<int>* passorder = new vector<int>();
    unordered_set<int>* passset = new unordered_set<int>();
    DFS(*tree,0,*passorder,*passset,edgelen,0);

    //Print out the order and the minimum
    for (int i = 1;i < ord->size();i++){
       cout<<ord->at(i);
       cout<<"\n";
    }
    cout<<mindist;


    if (!ord){
        delete ord;
    }
    delete tree;
    free(sortedge);
    for (int k = 0; k < n+1; ++k) {
        free(edgelen[k]);
    }
    free(edgelen);
    for (int l = 0; l < n+1; ++l) {
        delete arr[l];
    }
    free(arr);
}

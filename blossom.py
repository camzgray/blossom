#!/usr/bin/env python
# coding: utf-8

# In[1]:


# An implementation of the Blossom algorithm
# https://en.wikipedia.org/wiki/Blossom_algorithm

import copy


# In[2]:


class Graph:
    def __init__(self,graph_dict):
        self.graph_dict = graph_dict
    def __repr__(self):
        return str(self.graph_dict)
    def vertices(self):
        return self.graph_dict.keys()
    def edges(self):
        # currently returns just (i,j)
        edge_list = []
        for i in self.vertices():
            for j in self.graph_dict[i]:
                if i < j:
                    edge_list.append((i,j))
        return edge_list

class Node:
    def __init__(self,vertex,nodes=None):
        self.vertex = vertex # the name of the vertex
        if nodes is None:
            nodes = []
        self.nodes = nodes # a list of nodes that are below the node
    def __repr__(self):
        return str(self.vertex) + str(self.nodes)
    def vertices(self):
        #returns vertices of itself and its nodes
        vertexlist = [self.vertex]
        for n in self.nodes:
            vertexlist = vertexlist + n.vertices()
        return vertexlist
    def nodelist(self):
        # lists the nodes it is attached to and the nodes they are attached to
        nres = []
        for k in self.nodes:
            nres = nres + [k] + k.nodelist()
        return nres
    def distance_to(self,v):
        # distance to vertex v (assuming v is upwards)
        if self.vertex == v:
            return 0
        #elif len(self.nodes) == 0:
        #    return float('inf')
        else:
            minv = float('inf')
            for k in self.nodes:
                dtk = k.distance_to(v)
                if dtk < minv:
                    minv = dtk
            return 1+minv
    def findpath(self,v):
        if self.vertex == v:
            return [self.vertex]
        elif self.nodes == []:
            return []
        else:
            for k in self.nodes:
                fp = k.findpath(v)
                if fp != []:
                    return [self.vertex] + fp
            return []
        

class Tree:
    def __init__(self,root):
        self.root = root # a root node
    def __repr__(self):
        return str(self.root)
    def vertices(self):
        # lists the vertices
        return self.root.vertices()
    def nodelist(self):
        # lists the nodes
        return [self.root] + self.root.nodelist()
    def distance_to_root(self,v):
        # finds the distance to root from vertex name v
        return self.root.distance_to(v)
    def findpath(self,v):
        return self.root.findpath(v)

class Forest:
    def __init__(self,trees=None):
        if trees is None:
            trees = []
        self.trees = trees # list of trees
    def __repr__(self):
        return str(self.trees)
    def vertices(self):
        # lists the vertices
        vres = []
        for k in self.trees:
            vres = vres + k.vertices()
        return vres
    def nodelist(self):
        # lists the nodes
        nres = []
        for k in self.trees:
            nres = nres + k.nodelist()
        return nres
    def distance_to_root(self,v):
        # finds the distance to root from vertex name v
        minv = float('inf')
        for k in self.trees:
            dtk = k.distance_to_root(v)
            if dtk < minv:
                minv = dtk
        return minv
    def findroot(self,v):
        # returns the root node of vertex v
        for k in self.trees:
            if v in k.vertices():
                return k
    def findpath(self,v):
        # returns a list that is a path from a root to vertex v
        for k in self.trees:
            if v in k.vertices():
                return k.findpath(v)
        


# In[3]:


# using the pseudo-code from wikipedia on the blossom alg
def find_maximum_matching(G,M):
    # G is the graph, M is an initial matching
    P = find_augmenting_path(G,M)
    # P is a path - a list of vertices (subset of G's vertices)
    if len(P) != 0: #if P is non-empty
        return find_maximum_matching(G, augment(M,P))
    else:
        return M

def augment(M,P):
    #augments matching M along path P, returns new graph N
    N = copy.deepcopy(M)
    for i in range(len(P)-1):
        if P[i+1] in N.graph_dict[P[i]]:
            N.graph_dict[P[i]].remove(P[i+1])
            N.graph_dict[P[i+1]].remove(P[i])
        else:
            N.graph_dict[P[i]].append(P[i+1])
            N.graph_dict[P[i+1]].append(P[i])
    return N

def find_augmenting_path(G,M):
    #returns an augmenting path
    F = Forest([]) #empty forest
    markings = {}
    for i in G.vertices():
        markings[i] = False
    for i in G.edges():
        markings[i] = False
    for i in M.edges():
        markings[i] = True
    for v in G.vertices():
        if M.graph_dict[v] == []: #v is exposed
            (F.trees).append(Tree(Node(v)))
    uve = unmarked_vertices_even(F,markings)
    while len(uve) > 0:
        v = uve[0]
        umv = unmarked_edges(G,markings,v.vertex)
        while len(umv) > 0:
            e = umv[0]
            if e[0] == v.vertex:
                w = e[1]
            else:
                w = e[0]
            if not (w in F.vertices()):
                # (v,w) is marked so w is in M
                x = (M.graph_dict[w])[0]
                v.nodes.append(Node(w,[Node(x)])) # adds (w,x) and (v,w) to F
            else:
                if F.distance_to_root(w)%2 == 0:
                    v_root = F.findroot(v.vertex)
                    w_root = F.findroot(w)
                    if v_root.root.vertex != w_root.root.vertex:
                        P = F.findpath(v.vertex)
                        tempL = F.findpath(w)
                        tempL.reverse()
                        P = P + tempL
                        return P
                    else:
                        blossom = F.findpath(v.vertex)
                        tempL = F.findpath(w)
                        tempL.reverse()
                        blossom = blossom + tempL
                        newblossom = cut_edges(blossom)
                        # blossom should always have odd number of points
                        G2 = contract(G,newblossom)
                        M2 = contract(M,newblossom)
                        P2 = find_augmenting_path(G2,M2)
                        P = lift(P2,G,M,newblossom) # might not need all these
                        return P
            markings[e] = True # mark e
            umv = unmarked_edges(G,markings,v.vertex) # recompute the list
        markings[v.vertex] = True # mark v
        uve = unmarked_vertices_even(F,markings) # recompute the list
    P = []
    return P

def contract(G,vlist):
    # returns a new graph made from contracting a graph on vlist
    # all edges connecting to vlist vertices become connected to vlist[0], other vlist vertices removed 
    G2 = copy.deepcopy(G)
    if len(vlist)<2:
        return G2
    else:
        for k in vlist[1:]:
            for ke in G2.graph_dict[k]:
                if not (ke in G2.graph_dict[vlist[0]] or ke in vlist):
                    G2.graph_dict[vlist[0]].append(ke)
                    G2.graph_dict[ke].append(vlist[0]) # needs to be unique
                G2.graph_dict[ke].remove(k)
            if k in G2.graph_dict[vlist[0]]:
                G2.graph_dict[vlist[0]].remove(k)
            del G2.graph_dict[k]
        return G2

def cut_edges(blossom):
    # cuts the stem from a blossom: [1 2 3 4 5 2 1] -> [2 3 4 5]
    i = 0
    while blossom[i] == blossom[-1-i]:
        i = i + 1
    return blossom[i-1:-i]

def unmarked_vertices_even(F,markings):
    # finds unmarked vertices in F of even root distance, and returns a list of node
    rlist = []
    for v in F.nodelist():
        if (markings[v.vertex] == False) and (F.distance_to_root(v.vertex)%2 == 0):
            rlist.append(v)
    return rlist

def unmarked_edges(G,markings,v):
    # finds unmarked edges in G connected to v
    elist = []
    for w in G.graph_dict[v]:
        if v < w:
            test_edge = (v,w)
        else:
            test_edge = (w,v)
        if markings[test_edge] == False:
            elist.append(test_edge)
    return elist

def lift(P2,G,M,newblossom):
    # Lifts P2 to G
    contpoint = newblossom[0] #contracting vertex
    P = copy.deepcopy(P2)
    if contpoint in P:
        # P goes through the blossom - if it doesn't, lifts automatically
        bindex = P.index(contpoint) # index of the blossom
        if bindex == len(P)-1:
            # blossom is the right endpoint
            for b in range(len(newblossom)):
                if newblossom[b] in G.graph_dict[P[-2]]:
                    aindex = b
                if M.graph_dict[newblossom[b]] == []: # exposed vertex
                    cindex = b
        elif bindex == 0:
            # blossom is the left endpoint
            for b in range(len(newblossom)):
                if M.graph_dict[newblossom[b]] == []: # exposed vertex
                    aindex = b
                if newblossom[b] in G.graph_dict[P[1]]:
                    cindex = b
        else:
            # blossom is within P
            # What blossom points does P connect to originally?
            for b in range(len(newblossom)):
                if newblossom[b] in G.graph_dict[P[bindex-1]]:
                    aindex = b
                if newblossom[b] in G.graph_dict[P[bindex+1]]:
                    cindex = b
        pathindex = blossom_path(newblossom,aindex,cindex)
        path1 = []
        for i in pathindex[0]:
            path1.append(newblossom[i])
        path2 = []
        for i in pathindex[1]:
            path2.append(newblossom[i])
        if len(P)%2 == 0:
            # need path in blossom to be even number of edges (odd)
            if len(path1)%2 == 1:
                # use path1
                path = path1
            else:
                path = path2
        else:
            if len(path1)%2 == 1:
                # use path2
                path = path2
            else:
                path = path1
        P = P[:bindex] + path + P[bindex+1:]
    return P

def blossom_path(blossom,aindex,cindex):
    # finds two paths (indices) from a to c in the blossom, inclusive
    if aindex <= cindex:
        path1index = list(range(aindex,cindex+1))
        path2index = list(reversed(range(0,aindex+1))) + list(reversed(range(cindex,len(blossom))))
        return (path1index,path2index)
    elif aindex > cindex:
        path1index = list(reversed(range(cindex,aindex+1)))
        path2index = list(range(aindex,len(blossom))) + list(range(0,cindex+1))
        return (path1index,path2index)


# In[5]:


myGraph = Graph({0:[1,4],1:[0,2],2:[1,3],3:[2,4],4:[3,0,5],5:[4,6],6:[5]})
myMatch = Graph({0:[],1:[],2:[],3:[],4:[],5:[],6:[]})
myGraph2 = Graph({0:[1,4,7],1:[0,2],2:[1,3],3:[2,4,8],4:[3,0,5],5:[4,6],6:[5],7:[0],8:[3]})
myMatch2 = Graph({0:[],1:[],2:[],3:[],4:[],5:[],6:[],7:[],8:[]})
myGraph3 = Graph({0:[1,3],1:[0,2,3,4],2:[1,4],3:[0,1,4,5],4:[1,2,3,5],5:[3,4]})
myMatch3 = Graph({0:[1],1:[0],2:[4],3:[],4:[2],5:[]})
myGraph4 = Graph({0: [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15], 1: [2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15], 2: [1, 5, 7, 9, 11, 15], 3: [0, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15, 16], 4: [0, 1, 3, 13, 16], 5: [0, 1, 2, 3, 12, 13, 16], 6: [0, 1, 3, 13, 16], 7: [0, 1, 2, 3, 12, 13, 16], 8: [0, 1, 3, 13, 16], 9: [0, 1, 2, 3, 12, 13, 16], 10: [0, 1, 3, 13, 16], 11: [0, 1, 2, 3, 12, 13, 16], 12: [1, 5, 7, 9, 11, 15], 13: [0, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15, 16], 14: [0, 1, 3, 13, 16], 15: [0, 1, 2, 3, 12, 13, 16], 16: [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15]})
myMatch4 = Graph({0: [], 1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: [], 8: [], 9: [], 10: [], 11: [], 12: [], 13: [], 14: [], 15: [], 16: []})


# In[6]:


print(find_maximum_matching(myGraph,myMatch))
print(find_maximum_matching(myGraph2,myMatch2))
print(find_maximum_matching(myGraph3,myMatch3))
print(find_maximum_matching(myGraph4,myMatch4))


# In[ ]:





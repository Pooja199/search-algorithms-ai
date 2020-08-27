package com.usc;

import java.io.*;
import java.util.*;

class NodeObj implements Comparable<NodeObj>{
    int x;
    int y;
    int zValue;
    long g;
    long cost;
    NodeObj parent;

    NodeObj(int x, int y, int zValue){
        this.x=x;
        this.y=y;
        this.cost = 100000000000L;
        this.g=10000000000L;
        this.zValue = zValue;
    }

    @Override
    public int compareTo(NodeObj o) {
        if (this.cost < o.cost)
            return -1;
        if (this.cost > o.cost)
            return 1;
        return 0;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof NodeObj) {
            NodeObj node = (NodeObj) obj;
            return this.x == node.x && this.y == node.y;
        }
        return false;
    }
}

public class homework {
    private static int[] hx = {-1, -1, -1, 0, 0, 1, 1, 1};
    private static int[] wy = {-1, 0, 1, -1, 1, -1, 0, 1};
    private static int w, h, z, startW, startH, targets;
    private static List<NodeObj> targetList = new ArrayList<NodeObj>();

    public static void main(String[] args) throws IOException {
        File file = new File("input.txt");
        String algorithm = null;
        Scanner sc = null;
        try {
            sc = new Scanner(new FileInputStream(file));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        algorithm = sc.nextLine();
        w = sc.nextInt();
        h = sc.nextInt();
        startW = sc.nextInt();
        startH = sc.nextInt();
        z = sc.nextInt();
        targets = sc.nextInt();
        int t = targets;
        while (t > 0) {
            int y = sc.nextInt();
            int x = sc.nextInt();
            //todo check if 0 is ok
            targetList.add(new NodeObj(x,y,0));
            t--;
        }
        NodeObj[][] nodes = new NodeObj[h][w];
        for (int i=0; i < h; i++) {
            for(int j=0; j< w; j++){
                nodes[i][j]= new NodeObj(i, j, sc.nextInt());
            }
        }
        switch (algorithm) {
            case "BFS":
                calculateBFSPath(nodes);
                break;
            case "UCS":
                calculateUCSPath(nodes);
                break;
            case "A*":
                calculateAStar(nodes);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + algorithm);
        }

    }

    private static void printPaths() throws IOException {
        FileWriter fileWriter = new FileWriter("output.txt");
        PrintWriter printWriter = new PrintWriter(fileWriter);
        final int[] size = {targetList.size()};
        targetList.forEach( target -> {
            long cost = 0;
            if(target.x==startH && target.y==startW){
                if(size[0] >1)
                    printWriter.println(target.y + "," + target.x);
                else
                    printWriter.print(target.y + "," + target.x);
            }else if(target.parent==null){
                if(size[0] >1)
                    printWriter.println("FAIL");
                else
                    printWriter.print("FAIL");
            }else {
                Stack<NodeObj> stack = new Stack<>();
                System.out.println(target.parent.cost);
                while (target.parent != null) {
                    stack.add(target);
                    target = target.parent;
                }
                stack.add(target);
                while (!stack.empty()) {
                    NodeObj n = stack.pop();
                    printWriter.write(n.y + "," + n.x + " ");
                }if(size[0] >1) {
                    printWriter.println();
                }
                size[0]--;
            }
        });
        printWriter.close();
    }

    private static void calculateBFSPath(NodeObj[][] grid) throws IOException {
        LinkedList<NodeObj> queue = new LinkedList<>();
        int[][] isVisited = new int[h][w];
        int size = targetList.size();
        NodeObj landing = grid[startH][startW];
        landing.cost=0;
        isVisited[landing.x][landing.y] = 1;
        queue.add(landing);
        while (!queue.isEmpty()) {
            NodeObj node = queue.poll();
            for (NodeObj tar : targetList) {
                if (tar.x == node.x && tar.y == node.y) {
                    tar.parent = node.parent;
                    size--;
                }
            }
            if(size==0){
                break;
            }
            for (int movei = 0; movei < 8; movei++) {
                int row = hx[movei] + node.x;
                int col = wy[movei] + node.y;
                if (isValid(row,col) && Math.abs(grid[row][col].zValue-node.zValue)<=z && isVisited[row][col]==0) {
                    isVisited[row][col] = 1;
                    grid[row][col].parent=node;
                    queue.add(grid[row][col]);
                }
                //todo failure scenario
            }
        }
        printPaths();
    }

    private static void calculateAStar(NodeObj[][] grid) throws IOException {
        NodeObj landing = grid[startH][startW];
        int[][] isVisited = new int[h][w];
        PriorityQueue<NodeObj> priorityQueue = new PriorityQueue<>();
        Set<NodeObj> set = new HashSet<>();
        priorityQueue.add(landing);
        int size = targetList.size();
        NodeObj goal = nearestNeighbour();
        landing.g=0;
        landing.cost=heuristicFunction(landing,goal);
        while (!priorityQueue.isEmpty() && size>0){
            NodeObj node = priorityQueue.remove();
            isVisited[node.x][node.y]=1;
            for (NodeObj tar : targetList) {
                if (tar.x == node.x && tar.y == node.y) {
                    tar.parent = node.parent;
                    set.add(node);
                }
            }
//            if(size==set.size()){
//                break;
//            }
            for(int movei =0; movei <8; movei++){
                int row = hx[movei] + node.x;
                int col = wy[movei] + node.y;
                if(isValid(row,col) && Math.abs(grid[row][col].zValue-node.zValue)<=z && isVisited[row][col]==0){
                    //todo store parent
                    long distanceFromPtoC = calculateAStarPathCost(grid[row][col], node);
                    if(distanceFromPtoC < grid[row][col].g){
                        grid[row][col].g=distanceFromPtoC;
                        grid[row][col].cost = distanceFromPtoC + heuristicFunction(grid[row][col],goal);
                        grid[row][col].parent=node;
                        priorityQueue.add(grid[row][col]);
                    }
                }
            }
        }
        printPaths();
    }

    private static long calculateAStarPathCost(NodeObj child, NodeObj parent) {
        long distance;
        int zDiff = Math.abs(child.zValue - parent.zValue);
        if (Math.abs(parent.x - child.x) == Math.abs(parent.y - child.y)) {
            distance = parent.cost + 14 + zDiff;
        } else
            distance = parent.cost + 10 + zDiff;
        return distance;
    }

    private static void calculateUCSPath(NodeObj[][] grid) throws IOException {
        NodeObj landing = grid[startH][startW];
        int[][] isVisited = new int[h][w];
        landing.cost=0;
        PriorityQueue<NodeObj> priorityQueue = new PriorityQueue<>();
        priorityQueue.add(landing);
        int size = targetList.size();
        Set<NodeObj> set = new HashSet<>();
        while (!priorityQueue.isEmpty() && size>0){
            NodeObj node = priorityQueue.remove();
            isVisited[node.x][node.y]=1;
            for (NodeObj tar : targetList) {
                if (tar.x == node.x && tar.y == node.y) {
                    tar.parent = node.parent;
                    set.add(node);
                }
            }
            if(set.size()==size){
                break;
            }
            for(int movei =0; movei <8; movei++){
                int row = hx[movei] + node.x;
                int col = wy[movei] + node.y;
                if(isValid(row,col) && Math.abs(grid[row][col].zValue-node.zValue)<=z && isVisited[row][col]==0){
                    //todo store parent

                    long distanceFromPtoC = calculateUCSPathCost(node,grid[row][col]);
                    if(distanceFromPtoC < grid[row][col].cost){
                        grid[row][col].cost = distanceFromPtoC;
                        grid[row][col].parent=node;
                        priorityQueue.add(grid[row][col]);
                    }
                }
            }
        }
        printPaths();
    }

    private static Boolean isValid(int row, int col) {
        return (row >= 0) && (row < h) &&
                (col >= 0) && (col < w);
    }

    private static long calculateUCSPathCost(NodeObj parent, NodeObj child) {
        long distance;
        if (Math.abs(parent.x - child.x) == Math.abs(parent.y - child.y)) {
            distance = parent.cost + 14;
        } else
            distance = parent.cost + 10;
        return distance;
    }

    private static int heuristicFunction(NodeObj node, NodeObj goal) {
        int h = Math.max(Math.abs(node.x - goal.x), Math.abs(node.y - goal.y));
        return h;
    }

    private static NodeObj nearestNeighbour(){
        int minDist = 1000000000, minX=1000000000, minY=1000000000, minZ=1000000000;
        for (NodeObj target : targetList) {
            double distance = Math.sqrt(Math.pow(startH - target.x, 2) + Math.pow(startW - target.y, 2));
            if (distance < minDist) {
                minX = target.x;
                minY = target.y;
                minZ = target.zValue;
            }
        }
        NodeObj nodeObj = new NodeObj(minX,minY,minZ);
        return nodeObj;
    }
}

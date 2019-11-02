package org.firstinspires.ftc.teamcode.SkyStone.Parhfinding;



import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByEncoder_Linear;

import java.util.ArrayList;
import java.util.List;

/*
class Astar {

    final List<Node> open = null;
    final List<Node> closed = null;
    final List<Node> path = null;
    final int[][] maze = null;
    Node now;
    final int xstart=0;
    final int ystart =0;

    static class Node implements Comparable{
        public Node parent;
        public int x,y;
        public double g;
        public double h;
        Node(Node parent, int xpos, int ypos, double g, double h){
            this.parent = parent;
            this.x=xpos;
            this.y=ypos;
            this.g=g;
            this.h=h;
        }

        @Override
        public int compareTo(Object o){
            Node that = (Node) o;
            return (int)((this.g+this.h)-(that.g+that.h));
        }
    }

    AStar(int[][] maze, int xstart, int ystart, boolean diag){
        this.open = new ArrayList<>();
        this.closed = new ArrayList<>();
        this.path = new ArrayList<>();
        this.maze = maze;
        this.now = new Node(null,xstart,ystart,0,0);
        this.xstart = xstart;
        this.ystart=ystart;
        this.diag = diag;

    }

    public List<Node> findPathTo(int xend, int yend){
        this.xend = xend;
        this.yend = yend;
        this.closed.add(this.now);
        addNeighborsToOpenList();
        while(this.now.x != this.xend || this.now.y != this.yend){
            if(this.open.isEmpty()){
                return null;
            }
            this.now = this.open.get(0);
            this.open.remove(0);
            this.closed.add(this.now);
            addNeighborsToOpenList();
        }
        this.path.add(0,this.now);
        while(this.now.x != this.xstart||this.now.y != this.ystart) {
            this.now = this.now.parent;
            this.path.add(0, this.now);
        }
        return this.path;
    }

    private static boolean findNeighborsInList(List<Node> array, Node node){
        return array.stream().anyMatch((n) -> (n.x==node.x && n.y==node.y));
    }

    private double distance(int dx, int dy){
        if(this.diag){
            return Math.hypot(this.now.x+dx-this.xend,this.now.y+dy-this.yend);
        } else{
            return Math.abs(this.now.x + dx -this.xend) + Math.abs(this.now.y + dy - this.yend);
        }
    }
    private void addNeighborsToOpenList(){
        Node node;
        for(int x = -1; x <=1; x++){
            for(int y = -1; y <=1; y++){
                if(!this.diag && x != 0 && y !=0){
                    continue;
                }
                node = new Node(this.now,this.now.x,this.now.y+y,this.now.g,this.distance(x,y));
                if((x!=0 || y!=0)
                    && this.now.x + x >= 0 && this.now.x + x < this.maze[0].length
                    && this.now.y +y >= 0 && this.now.y +y < this.maze.length
                    && this.maze [this.now.y+y][
            }
        }
    }



}


 */
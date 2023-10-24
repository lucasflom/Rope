public class Sphere {
    public PVector pos;
    public float r;
    public float mass;
    public PVector vel;

    public Sphere(PVector pos, float r){
        this.pos = pos;
        this.r = r;
        this.mass = 9999;
        this.vel = new PVector(0,0,0);
    }
    
    public Sphere(PVector pos, float r, float mass, PVector vel){
        this.pos = pos;
        this.r = r;
        this.mass = mass;
        this.vel = vel;
    }
    

    // Sphere v Point
    public Boolean isColliding(PVector point){
        float distance = PVector.dist(point, this.pos);
        return (distance <= this.r);
    }
    
   	// // Circle v Circle
    // public Boolean isColliding(Circle other) {
    //     float distance = this.pos.distanceTo(other.pos);
    //     return (distance < this.r + other.r);
    // }
    
    // // Circle v Line Segment
    // public Boolean isColliding(Line line) {
    //     // if ((line.l1.distanceTo(this.pos)) <= this.r || line.l2.distanceTo(this.pos <= this.r)){
    //     //     return true;
    //     // }
    //     Vec2 closestPoint = this.closestPoint(line);
    //     return (this.pos.distanceTo(closestPoint) < this.r);
    // }

    // // Returns the closest point between the circle and a line
    // public Vec2 closestPoint(Line line) {
    //     float len = line.l1.minus(line.l2).length();
    //     float dot = (((this.pos.x-line.l1.x)*(line.l2.x-line.l1.x)) + ((this.pos.y-line.l1.y)*(line.l2.y-line.l1.y))) / pow(len, 2);
    //     Vec2 closest = new Vec2(line.l1.x + (dot *(line.l2.x-line.l1.x)), (line.l1.y + (dot * (line.l2.y - line.l1.y))));
    //     if (line.isColliding(closest)) return closest;
    //     // There is no point on the line that is closest so return whichever end point is closest
    //     if (line.l1.distanceTo(this.pos) < line.l2.distanceTo(this.pos)) return line.l1;
    //     return line.l2;
    // }

    // Circle v Box
    // public Boolean isColliding(Box box) {
    //     Vec2 closestPoint = new Vec2 (constrain(this.pos.x, box.pos.x - box.w/2, box.pos.x + box.w/2), constrain(this.pos.y, box.pos.y - box.h/2, box.pos.y + box.h/2));
    //     return (this.pos.distanceTo(closestPoint) < this.r);
    // }
    // public Boolean isColliding(Box box) {
    //     // Check if circle center is inside the box
    //     // Check start point
    //     if (box.isColliding(this.pos)) return true;
    //     // Check if the circle intersects any of the boxes lines
    //     Line b1 = new Line(box.pos.x - box.w/2, box.pos.y + box.h/2, box.pos.x + box.w/2, box.pos.y + box.h/2);
    //     Line b2 = new Line(box.pos.x - box.w/2, box.pos.y - box.h/2, box.pos.x + box.w/2, box.pos.y - box.h/2);
    //     Line b3 = new Line(box.pos.x - box.w/2, box.pos.y + box.h/2, box.pos.x - box.w/2, box.pos.y - box.h/2);
    //     Line b4 = new Line(box.pos.x + box.w/2, box.pos.y + box.h/2, box.pos.x + box.w/2, box.pos.y - box.h/2);
    //     if (this.isColliding(b1)) return true; 
    //     if (this.isColliding(b2)) return true;
    //     if (this.isColliding(b3)) return true;
    //     if (this.isColliding(b4)) return true;
        
    //     return false;
    // }

    // // Returns the closest point between the circle and a box (Probably should be better and is causing the sliding motion)
    // public Vec2 closestPoint(Box box) {
    //     return new Vec2 (constrain(this.pos.x, box.pos.x - box.w/2, box.pos.x + box.w/2), constrain(this.pos.y, box.pos.y - box.h/2, box.pos.y + box.h/2));
    // }
}
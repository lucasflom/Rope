// Node struct
public class Node {
  PVector pos;
  PVector vel;
  PVector last_pos;
  float h;      // The height of the node
  float mass;

  public Node(PVector pos) {
    this.pos = pos;
    this.vel = new PVector(0, 0, 0);
    this.last_pos = pos;
    this.h = (height - this.pos.y * scene_scale) / scene_scale;
    this.mass = 1;
  }
}

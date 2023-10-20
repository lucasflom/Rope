// Node struct
class Node {
  Vec2 pos;
  Vec2 vel;
  Vec2 last_pos;
  float h;      // The height of the node
  float mass;

  Node(Vec2 pos) {
    this.pos = pos;
    this.vel = new Vec2(0, 0);
    this.last_pos = pos;
    this.h = (height - this.pos.y * scene_scale) / scene_scale;
    this.mass = 1;
  }
}
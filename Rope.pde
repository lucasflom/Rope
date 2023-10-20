// Challenge:
//  - Move your simulation to 3D. You'll need to add a z coordinate to the nodes and links, and update the rendering
//    but the physics simulation should be the same!


// Link length
float link_length = 0.1;

// Nodes
int rope_length = 50;
Node[] node_list = new Node[rope_length];
Vec2 base_pos = new Vec2(5, 5);

// Obstacles
int num_obstacles = 1;
Circle[] obstacles = new Circle[num_obstacles];

// Gravity
Vec2 gravity = new Vec2(0, 10);

// Coefficient of Restitution
float cor = 0.75f;

// Scaling factor for the scene
float scene_scale = width / 10.0f;

// Physics Parameters
int relaxation_steps = 100;
int sub_steps = 10;

// File IO
String fname = "trial5_3.csv";
boolean add_data = false;


void setup() {
  size(900, 900);
  surface.setTitle("Double Pendulum");
  scene_scale = width / 10.0f;
  node_list[0] = new Node(base_pos);
  // Initialize the rope(s)
  for (int i = 1; i < rope_length; i++){
    Vec2 point = new Vec2((5 + (link_length * i)), 5);
    node_list[i] = new Node(point);
  }
  // Initialize the obtacles
  for (int i = 0; i < num_obstacles; i++){
    obstacles[i] = new Circle(new Vec2(3,7), 0.5);
  }
}


void update_physics(float dt) {
  // Semi-implicit Integration
  for (int i = 1; i < rope_length; i++){
    node_list[i].last_pos = node_list[i].pos;
    node_list[i].vel = node_list[i].vel.plus(gravity.times(dt));
    for (int j = 0; j < num_obstacles; j++){
      if (obstacles[j].isColliding(node_list[i].pos)){
        // Do collision thing
        Vec2 delta = node_list[i].pos.minus(obstacles[j].pos);
        Vec2 dir = delta.normalized();
        float v1 = dot(node_list[i].vel, dir);
        float v2 = dot(obstacles[j].vel, dir);
        float m1 = node_list[i].mass;
        float m2 = obstacles[j].mass;
        float nv1 = (m1 * v1 + m2 * v2 - m2 * (v1 - v2) * cor) / (m1 + m2);
        node_list[i].vel = node_list[i].vel.plus(dir.times(nv1 - v1));
        println("Node " + str(i) + " is colliding with obstacle " + str(j));
      }
    }
    node_list[i].pos = node_list[i].pos.plus(node_list[i].vel.times(dt));
  }

  // Constrain the distance between nodes to the link length
  for (int i = 0; i < relaxation_steps; i++) {
    for (int j = 1; j < rope_length; j++){
      Vec2 delta = node_list[j].pos.minus(node_list[j-1].pos);
      float delta_len = delta.length();
      float correction = delta_len - link_length;
      Vec2 delta_normalized = delta.normalized();
      node_list[j].pos = node_list[j].pos.minus(delta_normalized.times(correction / 2));
      node_list[j-1].pos = node_list[j-1].pos.plus(delta_normalized.times(correction / 2));
    }
    node_list[0].pos = base_pos; // Fix the base node in place
  }

  // Check for collisions with obstacles
  for (int i = 1; i < rope_length; i++){
    
  }

  // Update the velocities (PBD)
  for (int i = 0; i < rope_length; i++){
    node_list[i].vel = node_list[i].pos.minus(node_list[i].last_pos).times(1 / dt);
  }
}

boolean paused = false;

void keyPressed() {
  if (key == ' ') {
    paused = !paused;
  }
}

float time = 0;
void draw() {
  float dt = 1.0 / 30; //Dynamic dt: 1/frameRate;
  if (time >= 30) {
    paused = true;
    exit();
  }
  println(time);
  if (!paused) {
    for (int i = 0; i < sub_steps; i++) {
      time += dt / sub_steps;
      update_physics(dt / sub_steps);
    }
  }

  
  background(255);
  stroke(0);
  strokeWeight(2);
  println(nf(time, 0, 2));
  // obstacles
  fill(255, 0, 0);
  stroke(0);
  strokeWeight(0.02 * scene_scale);
  for (int i = 0; i < num_obstacles; i++){
   circle(obstacles[i].pos.x * scene_scale, obstacles[i].pos.y * scene_scale, obstacles[i].r * scene_scale);
  }

  // Draw Links (black)
  stroke(10);
  strokeWeight(0.02 * scene_scale);
  for (int i = 1; i < rope_length; i++){
    line(node_list[i-1].pos.x * scene_scale, node_list[i-1].pos.y * scene_scale, node_list[i].pos.x * scene_scale, node_list[i].pos.y * scene_scale);
  }
}  






// Helper information functions

float total_length_error() {
  float error = 0;
  for (int i = 1; i < rope_length; i++){
    Vec2 delta = node_list[i].pos.minus(node_list[i-1].pos);
    float delta_len = delta.length();
    float correction = delta_len - link_length;
    error += correction;
  }
  return error;
}

float total_energy() {
  // Compute the total energy (should be conserved)
  float kinetic_energy = 0;
  float potential_energy = 0;
  for (int i = 0; i < rope_length; i++){
    kinetic_energy += 0.5 * node_list[i].vel.lengthSqr(); // KE = (1/2) * m * v^2
    node_list[i].h = (height - node_list[i].pos.y * scene_scale) / scene_scale;
    potential_energy += gravity.y * node_list[i].h; // PE = m*g*h
  }


  float total_energy = kinetic_energy + potential_energy;
  return total_energy;

}

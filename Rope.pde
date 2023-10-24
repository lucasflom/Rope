//TODO:
// 1. Make the rope interact with itself while it moves
// 2. Add Multiple ropes moving in the scene
//   2.1 This should be done with a list of a list of rope nodes
// 3. Have the ropes interact, should be tied together with 1
// 4. Maybe add more obstacles?
// 
// Actually all of this should be cloth, could keep the rope just for something fun but it should all be converted to cloths
// Challenge:
//  - Move your simulation to 3D. You'll need to add a z coordinate to the nodes and links, and update the rendering
//    but the physics simulation should be the same!


// Link length
float link_length = 0.1;

// Nodes
int rope_length = 50;
Node[] node_list = new Node[rope_length];
PVector base_pos = new PVector(5, 5, 0);

// Obstacles
int num_obstacles = 1;
Sphere[] obstacles = new Sphere[num_obstacles];

// Gravity
PVector gravity = new PVector(0, 10, 0);

// Coefficient of Restitution
float cor = 0.75f;

// Scaling factor for the scene
float scene_scale = width / 10.0f;

// Physics Parameters
int relaxation_steps = 100;
int sub_steps = 10;

// Set up for making the rope spring like
float k = 2;
float kv = 0.1;


void setup() {
  size(900, 900);
  surface.setTitle("Rope Simulation");
  scene_scale = width / 10.0f;
  node_list[0] = new Node(base_pos.copy());
  // Initialize the rope(s)
  for (int i = 1; i < rope_length; i++){
    PVector point = new PVector((5 + (link_length * i)), 5, 0);
    node_list[i] = new Node(point);
  }
  // Initialize the obtacles
  for (int i = 0; i < num_obstacles; i++){
    obstacles[i] = new Sphere(new PVector(3 + (0.5 * i),7 + (0.5 * i)), 0.5);
  }
}


void update_physics(float dt) {
  // Semi-implicit Integration to update the velocity and move the points accordingly
  for (int i = 1; i < rope_length; i++){
    // Adding spring like motion
    boolean collided = false;
    PVector old_vel = node_list[i].vel.copy();
    float stringLen = PVector.dist(node_list[i].pos, node_list[i-1].pos);
    float stringF = -k*(stringLen - link_length);
    PVector string_dir = PVector.sub(node_list[i].pos, node_list[i-1].pos);
    string_dir.normalize();
    PVector dampF = PVector.mult(PVector.sub(node_list[i].vel,new PVector(1, 1)), -kv); // Dampening force, try messing with the values in the vector
    node_list[i].vel.add(PVector.mult(string_dir, dt));
    node_list[i].vel.add(PVector.mult(dampF, dt));
    node_list[i].last_pos = node_list[i].pos.copy();
    node_list[i].vel.add(PVector.mult(gravity, dt));

    // Obstacle collision detection
    for (int j = 0; j < num_obstacles; j++){
      float distance = PVector.dist(obstacles[j].pos, node_list[i].pos);
      if (distance <= obstacles[j].r){
        // Do collision thing
        PVector delta = PVector.sub(node_list[i].pos, obstacles[j].pos);
        PVector dir = delta.normalize(new PVector());
        float v1 = node_list[i].vel.dot(dir);
        PVector bounce = PVector.mult(dir, v1);
        node_list[i].vel.sub(PVector.mult(bounce, 1.5));
        node_list[i].pos.add(PVector.mult(dir, (0.001 + obstacles[j].r - distance)));
        collided = true;
      }
    }
    float vel_difference = PVector.sub(node_list[i].vel, old_vel).mag();
    node_list[i].pos.add(PVector.mult(node_list[i].vel, dt));
  }
  

  // Constrain the distance between nodes to the link length so that they don't move too far away from each other
  for (int i = 0; i < relaxation_steps; i++) {
    for (int j = 1; j < rope_length; j++){
      PVector delta = PVector.sub(node_list[j].pos, node_list[j-1].pos);
      float delta_len = delta.mag();
      float correction = delta_len - link_length;
      delta.normalize();
      node_list[j].pos.sub(PVector.mult(delta, (correction / 2)));
      node_list[j-1].pos.add(PVector.mult(delta, (correction / 2)));
    }
    node_list[0].pos = base_pos.copy(); // Fix the base node in place
  }


  // Update the velocities after constraining them back from the previous step(PBD)
  for (int i = 1; i < rope_length; i++){
    node_list[i].vel = PVector.mult(PVector.sub(node_list[i].pos, node_list[i].last_pos), 1/dt);
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
  float dt = 1.0 / 40; //Dynamic dt: 1/frameRate;
  if (time >= 30) {
    paused = true;
    exit();
  }
  if (!paused) {
    for (int i = 0; i < sub_steps; i++) {
      time += dt / sub_steps;
      update_physics(dt / sub_steps);
    }
  }

  
  background(255);
  stroke(0);
  strokeWeight(2);
  // obstacles
  fill(255, 0, 0);
  stroke(0);
  strokeWeight(0.02 * scene_scale);
  for (int i = 0; i < num_obstacles; i++){
   circle(obstacles[i].pos.x * scene_scale, obstacles[i].pos.y * scene_scale, obstacles[i].r * scene_scale * 2);
  }

  // Draw Links (black)
  stroke(10);
  strokeWeight(0.02 * scene_scale);
  for (int i = 1; i < rope_length; i++){
    line(node_list[i-1].pos.x * scene_scale, node_list[i-1].pos.y * scene_scale, node_list[i].pos.x * scene_scale, node_list[i].pos.y * scene_scale);
  }
}  






// Helper information functions

// float total_length_error() {
//   float error = 0;
//   for (int i = 1; i < rope_length; i++){
//     Vec2 delta = node_list[i].pos.minus(node_list[i-1].pos);
//     float delta_len = delta.length();
//     float correction = delta_len - link_length;
//     error += correction;
//   }
//   return error;
// }

// float total_energy() {
//   // Compute the total energy (should be conserved)
//   float kinetic_energy = 0;
//   float potential_energy = 0;
//   for (int i = 0; i < rope_length; i++){
//     kinetic_energy += 0.5 * node_list[i].vel.lengthSqr(); // KE = (1/2) * m * v^2
//     node_list[i].h = (height - node_list[i].pos.y * scene_scale) / scene_scale;
//     potential_energy += gravity.y * node_list[i].h; // PE = m*g*h
//   }


//   float total_energy = kinetic_energy + potential_energy;
//   return total_energy;

// }

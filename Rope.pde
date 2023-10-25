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


// Camera
Camera camera;

// Link length
float link_length = 0.1;
float cross_length = 0.245;
float link_width = 0.173;


// Nodes
int cloth_length = 50;
int cloth_width = 20;
Node[][] cloth_list = new Node[cloth_width][cloth_length];
PVector base_pos = new PVector(0, 0, -15);

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
int relaxation_steps = 10;
int sub_steps = 10;

// Set up for making the rope spring like
float k = 2;
float kv = 0.1;

int count = 0;

void setup() {
  size(900, 900, P3D);
  camera = new Camera();
  surface.setTitle("Rope Simulation");
  scene_scale = width / 10.0f;
  for (int j = 0; j < cloth_width; j++){
    // Set the headers
    cloth_list[j][0] = new Node(new PVector(base_pos.x + (j * link_length), base_pos.y + (j * link_length), base_pos.z + (j * link_length)));
    // Initialize the strands of the cloth
    // println(cloth_list[j][0].pos);
    for (int i = 1; i < cloth_length; i++){
      PVector point = new PVector((cloth_list[j][0].pos.x + (link_length * i)), cloth_list[j][0].pos.y, cloth_list[j][0].pos.z);
      cloth_list[j][i] = new Node(point);
      // if (i == (cloth_length-1) && j == (cloth_width / 2)) println(point);
    }
  }
  // Initialize the obtacles
  for (int i = 0; i < num_obstacles; i++){
    obstacles[i] = new Sphere(new PVector(3, 4.4, -14), 0.9);
  }
}


void update_physics(float dt) {
  // Semi-implicit Integration to update the velocity and move the points accordingly
  for (int r = 0; r < cloth_width; r++){
    for (int i = 0; i < cloth_length; i++){
      // Adding spring like motion
      if (i == 0) {

      } else {
        float stringLen = PVector.dist(cloth_list[r][i].pos, cloth_list[r][i-1].pos);
        float stringF = -k*(stringLen - link_length);
        PVector string_dir = PVector.sub(cloth_list[r][i].pos, cloth_list[r][i-1].pos);
        string_dir.normalize();
        cloth_list[r][i].vel.add(PVector.mult(string_dir, dt));
      }
      PVector dampF = PVector.mult(PVector.sub(cloth_list[r][i].vel,new PVector(1, 1)), -kv); // Dampening force, try messing with the values in the vector
      cloth_list[r][i].vel.add(PVector.mult(dampF, dt));
      cloth_list[r][i].last_pos = cloth_list[r][i].pos.copy();
      cloth_list[r][i].vel.add(PVector.mult(gravity, dt));
      // if (i == 0) println(cloth_list[r][i].vel);
      // Obstacle collision detection
      for (int j = 0; j < num_obstacles; j++){
        float distance = PVector.dist(obstacles[j].pos, cloth_list[r][i].pos);
        if (distance <= obstacles[j].r){
          // Do collision thing
          PVector delta = PVector.sub(cloth_list[r][i].pos, obstacles[j].pos);
          PVector dir = delta.normalize(new PVector());
          float v1 = cloth_list[r][i].vel.dot(dir);
          PVector bounce = PVector.mult(dir, v1);
          cloth_list[r][i].vel.sub(PVector.mult(bounce, 1.5));
          cloth_list[r][i].pos.add(PVector.mult(dir, (0.001 + obstacles[j].r - distance)));
        }
      }
    
    cloth_list[r][i].pos.add(PVector.mult(cloth_list[r][i].vel, dt));

    }
  }
  

  // Constrain the distance between nodes to the link length so that they don't move too far away from each other
  for (int i = 0; i < relaxation_steps; i++) {
    for (int r = 0; r < cloth_width; r++) {
      for (int j = 1; j < cloth_length; j++){
        // Check up and down
        PVector delta = PVector.sub(cloth_list[r][j].pos, cloth_list[r][j-1].pos);
        float delta_len = delta.mag();
        float correction = delta_len - link_length;
        delta.normalize();
        cloth_list[r][j].pos.sub(PVector.mult(delta, (correction / 2)));
        cloth_list[r][j-1].pos.add(PVector.mult(delta, (correction / 2)));
        // Check across
        if (r < (cloth_width - 1)){
          // Check the current node to the node to right of it
          delta = PVector.sub(cloth_list[r][j].pos, cloth_list[r+1][j].pos);
          delta_len = delta.mag();
          correction = delta_len - link_width;
          delta.normalize();
          cloth_list[r][j].pos.sub(PVector.mult(delta, (correction / 2)));
          cloth_list[r+1][j].pos.add(PVector.mult(delta, (correction / 2)));
        }
        cloth_list[r][0].pos = new PVector(base_pos.x + (r * link_length), base_pos.y + (r * link_length), base_pos.z + (r * link_length)); // Fix the base node in place
        
      }
      cloth_list[r][0].pos = new PVector(base_pos.x + (r * link_length), base_pos.y + (r * link_length), base_pos.z + (r * link_length)); // Fix the base node in place
      // println("Horizontal: " + PVector.dist(cloth_list[0][1].pos, cloth_list[1][1].pos));
      // println("Vertical: " + PVector.dist(cloth_list[0][1].pos, cloth_list[0][2].pos));
      // exit();
    }
  }
 
  

  // Update the velocities after constraining them back from the previous step(PBD)
  for (int r = 0; r < cloth_width; r++){
    for (int i = 1; i < cloth_length; i++){
      cloth_list[r][i].vel = PVector.mult(PVector.sub(cloth_list[r][i].pos, cloth_list[r][i].last_pos), 1/dt);
    }
  }
  
  

}

boolean paused = true;

void mousePressed() {
  // Update the obstacle
  float x = mouseX / scene_scale;
  float y = mouseY / scene_scale;
  println("Before: " + obstacles[0].pos);
  obstacles[0].pos = new PVector(x, y, obstacles[0].pos.z);
  println("After: " + obstacles[0].pos);
}

void keyPressed() {
  if (key == ' ') {
    paused = !paused;
  }
  camera.HandleKeyPressed();
}

void keyReleased() {
  camera.HandleKeyReleased();
}

float time = 0;

void draw() {
  float dt = 1.0 / 90; //Dynamic dt: 1/frameRate;
  camera.Update(dt);
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
  lights();
  stroke(0);
  strokeWeight(2);
  // obstacles
  fill(255, 0, 0);
  stroke(0);
  strokeWeight(0.02 * scene_scale);
  noStroke();
  for (int i = 0; i < num_obstacles; i++){
    pushMatrix();
    translate(obstacles[i].pos.x * scene_scale, obstacles[i].pos.y * scene_scale, obstacles[i].pos.z * scene_scale);
    sphere(obstacles[i].r * scene_scale);
    popMatrix();
  }

  // Draw Links (black)
  stroke(10);
  strokeWeight(0.02 * scene_scale);
  for (int r = 0; r < cloth_width; r++){
      // pushMatrix();
      // translate(cloth_list[r][0].pos.x * scene_scale, cloth_list[r][0].pos.y * scene_scale, cloth_list[r][0].pos.z * scene_scale);
      // sphere(1);
      // popMatrix();
    for (int i = 0; i < cloth_length; i++){
      if (r > 0) line(cloth_list[r-1][i].pos.x * scene_scale, cloth_list[r-1][i].pos.y * scene_scale, cloth_list[r-1][i].pos.z * scene_scale, cloth_list[r][i].pos.x * scene_scale, cloth_list[r][i].pos.y * scene_scale, cloth_list[r][i].pos.z * scene_scale);
      if (i > 0) line(cloth_list[r][i-1].pos.x * scene_scale, cloth_list[r][i-1].pos.y * scene_scale, cloth_list[r][i-1].pos.z * scene_scale, cloth_list[r][i].pos.x * scene_scale, cloth_list[r][i].pos.y * scene_scale, cloth_list[r][i].pos.z * scene_scale);
    }
  }
  
}  






// Helper functions

// Boolean sameSide(PVector p1, PVector p2, PVector o1, PVector o2){
//   PVector cp1 = PVector.cross(PVector.sub(p2, p1), PVector.sub(o1, p1), new PVector());
//   PVector cp2 = PVector.cross(PVector.sub(p2, p1), PVector.sub(o2, p1), new PVector());
//   return ((cp1.z * cp2.z) >= 0 && (cp1.y * cp2.y) >= 0 && (cp1.x * cp2.x) >= 0); // May need to be fixed for if 3D
// }

// float total_length_error() {
//   float error = 0;
//   for (int i = 1; i < cloth_length; i++){
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
//   for (int i = 0; i < cloth_length; i++){
//     kinetic_energy += 0.5 * node_list[i].vel.lengthSqr(); // KE = (1/2) * m * v^2
//     node_list[i].h = (height - node_list[i].pos.y * scene_scale) / scene_scale;
//     potential_energy += gravity.y * node_list[i].h; // PE = m*g*h
//   }


//   float total_energy = kinetic_energy + potential_energy;
//   return total_energy;

// }

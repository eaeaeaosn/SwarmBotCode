import processing.serial.*;

Serial myPort;
float roll = 0, pitch = 0, yaw = 0;
int dataCount = 0;

void setup() {
    size(700, 500, P3D);
    
    println("=== ICM-20948 Visualizer ===");
    println("Coordinate System: X=Right, Y=Forward, Z=Up");
    println("Available ports:");
    printArray(Serial.list());
    
    try {
        myPort = new Serial(this, "COM6", 115200);
        myPort.bufferUntil('\n');
        println("✓ Connected to COM6");
    } catch (Exception e) {
        println("✗ Failed to connect: " + e);
    }
}

void draw() {
    background(50);
    lights();
    
    // Camera position - viewing from top-right
    camera(0, -400, 200, 0, 0, 0, 0, 0, -1);
    
    // Rotate the entire scene 90 degrees to reorient axes
    rotateZ(radians(90));  // This makes X point into screen, Y point left
    
    // Apply rotations according to ICM-20948 coordinate system
    // Note: Processing's Y-axis points down, needs adjustment
    rotateZ(radians(-yaw));      // Yaw: around Z-axis (up/down)
    rotateY(radians(pitch));     // Pitch: around Y-axis (forward/back)
    rotateX(radians(-roll));     // Roll: around X-axis (left/right)
    
    // Draw sensor board (flat, Z-axis pointing up)
    pushMatrix();
    fill(0, 150, 255);
    box(120, 80, 20);  // Width(X) x Length(Y) x Thickness(Z)
    popMatrix();
    
    // Draw coordinate axes
    strokeWeight(3);
    
    // X-axis - Red - pointing right
    stroke(255, 0, 0);
    line(0, 0, 0, 100, 0, 0);
    pushMatrix();
    translate(110, 0, 0);
    fill(255, 0, 0);
    textSize(16);
    text("X", 0, 0);
    popMatrix();
    
    // Y-axis - Green - pointing forward
    stroke(0, 255, 0);
    line(0, 0, 0, 0, -100, 0);  // Changed: 0, 100, 0 → 0, -100, 0
    pushMatrix();
    translate(0, -110, 0);  // Changed: 0, 110, 0 → 0, -110, 0
    fill(0, 255, 0);
    text("Y", 0, 0);
    popMatrix();
    
    // Z-axis - Blue - pointing up
    stroke(0, 0, 255);
    line(0, 0, 0, 0, 0, 100);
    pushMatrix();
    translate(0, 0, 110);
    fill(0, 0, 255);
    text("Z", 0, 0);
    popMatrix();
    
    // Draw arrow on board indicating forward direction (Y-axis)
    pushMatrix();
    translate(0, -30, 10);  // Changed: 0, 30, 10 → 0, -30, 10
    fill(255, 255, 0);
    // Triangle arrow
    beginShape();
    vertex(0, -10, 0);  // Changed: 0, 10, 0 → 0, -10, 0
    vertex(-5, 5, 0);   // Changed: -5, -5, 0 → -5, 5, 0
    vertex(5, 5, 0);    // Changed: 5, -5, 0 → 5, 5, 0
    endShape(CLOSE);
    popMatrix();
    
    // Display data (2D text, not rotated)
    camera();  // Reset camera to 2D mode
    fill(255);
    textSize(16);
    text("Roll (X-axis): " + nf(roll, 0, 1) + "° (Left/Right tilt)", 10, 20);
    text("Pitch (Y-axis): " + nf(pitch, 0, 1) + "° (Forward/Back tilt)", 10, 40);
    text("Yaw (Z-axis): " + nf(yaw, 0, 1) + "° (Horizontal rotation)", 10, 60);
    text("Data packets: " + dataCount, 10, 80);
    
    // Coordinate system description
    textSize(12);
    fill(200);
    text("Coordinate System: X→Right  Y→Forward  Z→Up", 10, height - 10);
}

void serialEvent(Serial p) {
    try {
        String data = p.readStringUntil('\n');
        if (data != null) {
            data = trim(data);
            println("[" + dataCount + "] " + data);
            
            String[] values = split(data, ',');
            if (values.length == 3) {
                roll = float(values[0]);
                pitch = float(values[1]);
                yaw = float(values[2]);
                dataCount++;
            }
        }
    } catch (Exception e) {
        println("Parse error: " + e);
    }
}

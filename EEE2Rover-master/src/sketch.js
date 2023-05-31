let rez = 20
let rows, cols

let field = []
let button
let backgroundButton
let increment = 0.1;
let zoff = 0;
let noise;

let backGroundColor = 0


function setup() {
  createCanvas(1920,1080);
  cols = 1 + width/rez
  rows = 1 + height/rez
  
  noise = new OpenSimplexNoise(1)
  for(let i = 0; i < cols; i++){
    field[i] = []
    for(let j = 0; j < rows; j++){
      field[i].push(floor(random(2)))
    }
  }
  mysolution = false
}


function draw() {
    background(backGroundColor)
   // frameRate(2) 
   // for(let i = 0; i < cols; i++){
   //    field[i] = []
   //    for(let j = 0; j < rows; j++){
   //      field[i].push(floor(random(2)))
   //    }
   //  }
  
  let xoff = 0;
  for (let i = 0; i < cols; i++) {
    xoff += increment;
    let yoff = 0;
    for (let j = 0; j < rows; j++) {
      field[i][j] = float(noise.noise3D(xoff,     yoff, zoff));
      yoff += increment;
    }
  }
  zoff += 0.02;
  
  
  for(let i = 0; i < cols; i++){
    for(let j = 0; j < rows; j++){
      stroke(field[i][j] *255)
      strokeWeight(rez*0.4)
      point(i * rez, j * rez)
    }
  }
  
  if (mysolution){
      for(let i = 0; i < cols -1; i++){
      for(let j = 0; j < rows -1  ; j++){
        //stroke(field[i][j] *255)
        let x = i * rez
        let y = j * rez
        stroke(abs(255- backGroundColor))
        strokeWeight(rez*0.08)
        let center = createVector(x + 0.5 * rez, y + 0.5 * rez)
        let a = createVector(x , y + 0.5 * rez)
        let b = createVector(x + 0.5 * rez, y)
        let c = createVector(x + rez, y + 0.5 * rez)
        let d = createVector(x + rez, y + rez)

        if( ceil(field[i+1][j]) != ceil(field[i][j]) ){
          line(b.x, b.y, center.x, center.y)
        }


        if( ceil(field[i][j+1]) != ceil(field[i][j]) ){
          line(a.x, a.y, center.x, center.y)
        }
      }
    }
  
  } else {
    for(let i = 0; i < cols -1; i++){
        for(let j = 0; j < rows -1  ; j++){
          //stroke(field[i][j] *255)
          let x = i * rez
          let y = j * rez
          stroke(abs(255-backGroundColor))
          strokeWeight(rez*0.08)
         
          let a = createVector(x , y + 0.5 * rez)
          let b = createVector(x + 0.5 * rez, y)
          let c = createVector(x + rez, y + 0.5 * rez)
          let d = createVector(x + 0.5 * rez, y + rez)
          
          //let state = getState(field[i][j], field[i+1][j], field[i][j+1], field[i+1][j+1])
          
          if (ceil(field[i+1][j])!= ceil(field[i][j])){
            
            if (ceil(field[i][j+1])!= ceil(field[i][j])){
              
              if (ceil(field[i+1][j+1])!= ceil(field[i][j])){
                line(a.x, a.y, b.x, b.y)
              } else {
                line(a.x, a.y, b.x, b.y)
                line(d.x, d.y, c.x, c.y)
              }
              
            } else {
              if (ceil(field[i+1][j+1])!= ceil(field[i][j])){
                  line(b.x, b.y, d.x, d.y)
                } else {
                  line(b.x, b.y, c.x, c.y)
                }

            } 
            
          } else {
            if (ceil(field[i][j+1])!= ceil(field[i][j])){

                if (ceil(field[i+1][j+1])!= ceil(field[i][j])){
                  line(a.x, a.y, c.x, c.y)
                } else {
                  line(a.x, a.y, d.x, d.y)
      
                }

              } else {
                if (ceil(field[i+1][j+1])!= ceil(field[i][j])){
                    line(d.x, d.y, c.x, c.y)
                  } else {
                    // do nothing
                  }


          }      
        }
    }
    }
  }
}
// function getState(a, b, c, d) {
//   return a * 1 + b * 2 + c * 4 + d * 8
// }
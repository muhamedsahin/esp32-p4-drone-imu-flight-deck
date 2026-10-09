import {multiply,quaternionMatrix,WORLD_TO_VIEW,perspective,lookAt,slerp} from './math.js';

// Küçük, bağımsız WebGL çizici. Model/ışıklar yereldir; internetten model indirilmez.
export class DroneRenderer {
  constructor(canvas) {
    this.canvas=canvas;this.gl=canvas.getContext('webgl',{alpha:true,antialias:true});
    if(!this.gl)throw new Error('Bu tarayıcıda WebGL kullanılamıyor.');
    const gl=this.gl;
    const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(s));return s;};
    this.program=gl.createProgram();
    gl.attachShader(this.program,shader(gl.VERTEX_SHADER,'attribute vec3 p;attribute vec3 n;attribute vec3 c;uniform mat4 model;uniform mat4 vp;varying vec3 color;varying vec3 normal;void main(){gl_Position=vp*model*vec4(p,1.0);color=c;normal=mat3(model)*n;}'));
    gl.attachShader(this.program,shader(gl.FRAGMENT_SHADER,'precision mediump float;varying vec3 color;varying vec3 normal;void main(){float light=length(normal)<0.1?1.0:0.42+0.58*max(dot(normalize(normal),normalize(vec3(0.4,1.0,0.6))),0.0);gl_FragColor=vec4(color*light,1.0);}'));
    gl.linkProgram(this.program);if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw new Error('WebGL shader bağlanamadı.');
    this.model=gl.getUniformLocation(this.program,'model');this.vp=gl.getUniformLocation(this.program,'vp');
    this.attributes=['p','n','c'].map(name=>gl.getAttribLocation(this.program,name));
    this.drone=this.createDrone();this.grid=this.createGrid();this.axis=this.createAxes();
    this.q=[1,0,0,0];this.target=this.q;this.axes=true;this.reset();
    gl.enable(gl.DEPTH_TEST);
    let pointer=null;
    canvas.addEventListener('pointerdown',e=>{pointer={x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(!pointer)return;this.azimuth-=(e.clientX-pointer.x)*.008;this.elevation=Math.max(.06,Math.min(1.54,this.elevation+(e.clientY-pointer.y)*.008));pointer={x:e.clientX,y:e.clientY};});
    canvas.addEventListener('pointerup',()=>pointer=null);canvas.addEventListener('pointercancel',()=>pointer=null);
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.distance=Math.max(4,Math.min(12,this.distance+e.deltaY*.006));},{passive:false});
    canvas.addEventListener('webglcontextlost',e=>{e.preventDefault();this.lost=true;});
    canvas.addEventListener('webglcontextrestored',()=>location.reload());
    this.previous=performance.now();this.frame=this.frame.bind(this);requestAnimationFrame(this.frame);
  }
  reset(){this.azimuth=.76;this.elevation=.55;this.distance=7.3;}
  top(){this.azimuth=0;this.elevation=1.54;this.distance=7;}
  setQuaternion(q){this.target=q;}
  mesh(data,mode){const gl=this.gl,buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(data),gl.STATIC_DRAW);return {buffer,count:data.length/9,mode:mode??gl.TRIANGLES};}
  createDrone(){
    const data=[],ink=[.17,.21,.23],edge=[.31,.37,.39],lime=[.71,.9,.37],silver=[.68,.72,.69];
    const vertex=(p,n,c)=>data.push(...p,...n,...c);
    const box=(center,size,color,angle=0)=>{
      const ca=Math.cos(angle),sa=Math.sin(angle),transform=v=>[center[0]+ca*v[0]-sa*v[1],center[1]+sa*v[0]+ca*v[1],center[2]+v[2]];
      const normal=v=>[ca*v[0]-sa*v[1],sa*v[0]+ca*v[1],v[2]];
      const corners=[[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]].map(v=>transform(v.map((x,i)=>x*size[i]/2)));
      for(const [ids,n] of [[[0,3,2,1],[0,0,-1]],[[4,5,6,7],[0,0,1]],[[0,1,5,4],[0,-1,0]],[[3,7,6,2],[0,1,0]],[[1,2,6,5],[1,0,0]],[[0,4,7,3],[-1,0,0]]])for(const i of [0,1,2,0,2,3])vertex(corners[ids[i]],normal(n),color);
    };
    const cylinder=(center,radius,height,color,segments=40)=>{
      for(let i=0;i<segments;i++){
        const a=i/segments*Math.PI*2,b=(i+1)/segments*Math.PI*2;
        const point=(t,z)=>[center[0]+radius*Math.cos(t),center[1]+radius*Math.sin(t),center[2]+z];
        const n=t=>[Math.cos(t),Math.sin(t),0];
        for(const [t,z] of [[a,-height/2],[b,-height/2],[b,height/2],[a,-height/2],[b,height/2],[a,height/2]])vertex(point(t,z),n(t),color);
        for(const z of [-height/2,height/2])for(const p of [[center[0],center[1],center[2]+z],point(a,z),point(b,z)])vertex(p,[0,0,z>0?1:-1],color);
      }
    };
    for(const x of [-1,1])for(const y of [-1,1]){
      const mx=x*1.04,my=y*.85;
      box([mx/2,my/2,-.06],[Math.hypot(mx,my),.11,.13],ink,Math.atan2(my,mx));
      cylinder([mx,my,.02],.18,.22,edge);
      cylinder([mx,my,.16],.13,.10,x>0?lime:silver);
      cylinder([mx,my,.23],.055,.08,ink,20);
      // Pervaneler statik: panel motor devri ölçmüyor, dönüş animasyonu üretmiyoruz.
      box([mx,my,.28],[.86,.12,.027],x>0?lime:[.42,.47,.46],x*y*.4);
      box([mx,my,.28],[.12,.86,.027],x>0?lime:[.42,.47,.46],x*y*.4);
      box([mx,my,-.24],[.1,.09,.22],ink);
    }
    box([0,0,.03],[.92,.51,.25],ink);
    box([-.05,0,.2],[.62,.39,.15],edge);
    box([.3,0,.24],[.12,.4,.055],lime);
    box([-.12,0,.31],[.35,.3,.07],ink);
    box([.49,0,.045],[.06,.20,.16],silver);
    cylinder([.52,0,.13],.05,.025,lime,20);
    for(const y of [-1,1])box([-.08,y*.21,.015],[.53,.017,.018],silver);
    return this.mesh(data);
  }
  createGrid(){const d=[],c=[.83,.86,.8];for(let i=-7;i<=7;i++){d.push(-7,0,i,0,0,0,...c,7,0,i,0,0,0,...c,i,0,-7,0,0,0,...c,i,0,7,0,0,0,...c);}return this.mesh(d,this.gl.LINES);}
  createAxes(){const d=[];for(const [end,c] of [[[1.8,0,0],[.8,.4,.32]],[[0,1.6,0],[.2,.62,.43]],[[0,0,1.25],[.35,.53,.81]]])d.push(0,0,.35,0,0,0,...c,...end,0,0,0,...c);return this.mesh(d,this.gl.LINES);}
  draw(mesh,model){const gl=this.gl;gl.uniformMatrix4fv(this.model,false,model);gl.bindBuffer(gl.ARRAY_BUFFER,mesh.buffer);this.attributes.forEach((id,i)=>{gl.enableVertexAttribArray(id);gl.vertexAttribPointer(id,3,gl.FLOAT,false,36,i*12);});gl.drawArrays(mesh.mode,0,mesh.count);}
  frame(time){
    requestAnimationFrame(this.frame);if(this.lost||document.hidden)return;
    const gl=this.gl,dt=Math.min(.1,(time-this.previous)/1000);this.previous=time;
    const reduce=matchMedia('(prefers-reduced-motion: reduce)').matches;
    this.q=reduce?this.target:slerp(this.q,this.target,1-Math.exp(-dt*22));
    const rect=this.canvas.getBoundingClientRect(),ratio=Math.min(devicePixelRatio||1,2);
    const width=Math.round(rect.width*ratio),height=Math.round(rect.height*ratio);if(!width||!height)return;
    if(this.canvas.width!==width||this.canvas.height!==height){this.canvas.width=width;this.canvas.height=height;}
    gl.viewport(0,0,width,height);gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);
    const r=this.distance*Math.cos(this.elevation),eye=[r*Math.sin(this.azimuth),1.1+this.distance*Math.sin(this.elevation),r*Math.cos(this.azimuth)];
    gl.uniformMatrix4fv(this.vp,false,multiply(perspective(width/height),lookAt(eye,[0,1.1,0])));
    const identity=new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);this.draw(this.grid,identity);
    const model=multiply(WORLD_TO_VIEW,quaternionMatrix(this.q));model[13]=1.6;
    this.draw(this.drone,model);if(this.axes)this.draw(this.axis,model);
  }
}

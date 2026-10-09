// Hamilton quaternion [w,x,y,z]; sensördeki body -> world kuralı korunur.
export function normalize(q) {
  if (!Array.isArray(q) || q.length !== 4 || !q.every(Number.isFinite)) return null;
  const n = Math.hypot(...q);
  return n > 1e-9 ? q.map(v => v / n) : null;
}
export function fromEuler(roll, pitch, yaw) {
  const cr=Math.cos(roll/2),sr=Math.sin(roll/2),cp=Math.cos(pitch/2),sp=Math.sin(pitch/2),cy=Math.cos(yaw/2),sy=Math.sin(yaw/2);
  return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy];
}
export function slerp(a,b,t) {
  let dot=a.reduce((sum,v,i)=>sum+v*b[i],0);
  if(dot<0){b=b.map(v=>-v);dot=-dot;} // q ve -q aynı duruştur; kısa yolu seç.
  if(dot>.9995)return normalize(a.map((v,i)=>v+(b[i]-v)*t));
  const theta=Math.acos(Math.min(1,dot)),s=Math.sin(theta);
  return a.map((v,i)=>(Math.sin((1-t)*theta)*v+Math.sin(t*theta)*b[i])/s);
}
export function multiply(a,b) {
  const result=new Float32Array(16);
  for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)result[c*4+r]+=a[k*4+r]*b[c*4+k];
  return result;
}
export function quaternionMatrix([w,x,y,z]) {
  return new Float32Array([1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y),0,
    2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x),0,
    2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y),0,0,0,0,1]);
}
// Dünya Z yukarı -> WebGL sahnesinde Y yukarı. Bu bir sağ elli dönüşümdür.
export const WORLD_TO_VIEW=new Float32Array([1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1]);
export function perspective(aspect) {
  const f=1/Math.tan(Math.PI/8),near=.1,far=100;
  return new Float32Array([f/aspect,0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0]);
}
export function lookAt(eye,target) {
  const unit=v=>{const n=Math.hypot(...v);return v.map(x=>x/n);};
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const z=unit(eye.map((v,i)=>v-target[i])),x=unit(cross([0,1,0],z)),y=cross(z,x);
  const dot=a=>-a.reduce((s,v,i)=>s+v*eye[i],0);
  return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,dot(x),dot(y),dot(z),1]);
}

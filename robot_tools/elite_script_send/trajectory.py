#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹记录：CSV + 离线可打开的 3D HTML。"""
from __future__ import print_function

import csv
import json
import time
from pathlib import Path

from state_listener import format_pose

CSV_FIELDS = [
    "t_s",
    "x_mm", "y_mm", "z_mm",
    "rx_deg", "ry_deg", "rz_deg",
    "j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg",
]

HTML_MAX_POINTS = 8000


def pose_sample(parsed, t0, now=None):
    now = time.time() if now is None else now
    shown = format_pose(parsed)
    xyz = shown.get("xyz_mm") or (None, None, None)
    rpy = shown.get("rpy_deg") or (None, None, None)
    joints = shown.get("joints_deg") or (None,) * 6
    return {
        "t_s": now - t0,
        "x_mm": xyz[0], "y_mm": xyz[1], "z_mm": xyz[2],
        "rx_deg": rpy[0], "ry_deg": rpy[1], "rz_deg": rpy[2],
        "j1_deg": joints[0], "j2_deg": joints[1], "j3_deg": joints[2],
        "j4_deg": joints[3], "j5_deg": joints[4], "j6_deg": joints[5],
    }


def _fmt(value, digits):
    if value is None:
        return ""
    return ("%." + str(digits) + "f") % value


def write_csv(path, samples):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in samples:
            writer.writerow({
                "t_s": _fmt(row.get("t_s"), 3),
                "x_mm": _fmt(row.get("x_mm"), 3),
                "y_mm": _fmt(row.get("y_mm"), 3),
                "z_mm": _fmt(row.get("z_mm"), 3),
                "rx_deg": _fmt(row.get("rx_deg"), 4),
                "ry_deg": _fmt(row.get("ry_deg"), 4),
                "rz_deg": _fmt(row.get("rz_deg"), 4),
                "j1_deg": _fmt(row.get("j1_deg"), 4),
                "j2_deg": _fmt(row.get("j2_deg"), 4),
                "j3_deg": _fmt(row.get("j3_deg"), 4),
                "j4_deg": _fmt(row.get("j4_deg"), 4),
                "j5_deg": _fmt(row.get("j5_deg"), 4),
                "j6_deg": _fmt(row.get("j6_deg"), 4),
            })
    return path


def _downsample(samples, limit):
    if len(samples) <= limit:
        return samples
    step = float(len(samples) - 1) / float(limit - 1)
    out = []
    for i in range(limit):
        out.append(samples[int(round(i * step))])
    return out


def write_html(path, samples):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = []
    for row in _downsample(samples, HTML_MAX_POINTS):
        x, y, z = row.get("x_mm"), row.get("y_mm"), row.get("z_mm")
        if x is None or y is None or z is None:
            continue
        pts.append({
            "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
            "t": round(row.get("t_s") or 0, 3),
            "rx": round(row.get("rx_deg") or 0, 3),
            "ry": round(row.get("ry_deg") or 0, 3),
            "rz": round(row.get("rz_deg") or 0, 3),
            "j": [
                round(row.get("j1_deg") or 0, 3),
                round(row.get("j2_deg") or 0, 3),
                round(row.get("j3_deg") or 0, 3),
                round(row.get("j4_deg") or 0, 3),
                round(row.get("j5_deg") or 0, 3),
                round(row.get("j6_deg") or 0, 3),
            ],
        })
    html = _HTML_TEMPLATE.replace("__POINTS__", json.dumps(pts, ensure_ascii=False))
    html = html.replace("__COUNT__", str(len(samples)))
    html = html.replace("__SHOWN__", str(len(pts)))
    html = html.replace(
        "__STATE__",
        json.dumps(
            {"marks": [], "yaw": 0.75, "pitch": 0.5, "dist": 1, "axes": False, "data": False, "marking": False},
            ensure_ascii=False,
        ),
    )
    path.write_text(html, encoding="utf-8")
    return path


_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>Elite TCP 三维轨迹</title>
<style>
  html,body{margin:0;height:100%;width:100%;overflow:hidden;background:#111;color:#eee;font-family:Microsoft YaHei,Consolas,sans-serif}
  #bar{padding:0 12px;background:#1c1c1c;font-size:13px;display:flex;gap:16px;align-items:center;height:40px;box-sizing:border-box;white-space:nowrap;overflow:hidden}
  #bar label{cursor:pointer;user-select:none}
  #bar button{background:#333;color:#eee;border:1px solid #555;padding:2px 10px;cursor:pointer;border-radius:3px}
  #bar input[type=text]{width:96px;background:#222;color:#eee;border:1px solid #555;padding:2px 6px;border-radius:3px}
  #markTools{display:none;align-items:center;gap:8px}
  #wrap{display:flex;height:calc(100% - 40px)}
  canvas{flex:1;display:block;min-width:0;cursor:grab;background:#111}
  #side{width:320px;max-width:42vw;background:#181818;border-left:1px solid #333;padding:10px;overflow:auto;font-size:12px}
  #side.hidden{display:none}
  #hud{color:#ffcc02;min-height:4.5em;white-space:pre-wrap}
  table{width:100%;border-collapse:collapse;margin-top:8px}
  th,td{border-bottom:1px solid #333;padding:3px 4px;text-align:right;font-variant-numeric:tabular-nums}
  th{color:#9e9e9e;position:sticky;top:0;background:#181818}
  tr.sel td{background:#333;color:#ffcc02}
  h3{margin:10px 0 6px;font-size:13px;color:#bbb;font-weight:600}
</style>
</head>
<body>
<div id="bar">
  <span>TCP 轨迹（mm） 记录 __COUNT__ 点 / 显示 __SHOWN__ 点 · 拖动旋转 · 悬停看点</span>
  <label><input type="checkbox" id="chkAxes"/> 坐标系</label>
  <label><input type="checkbox" id="chkData"/> 坐标数据</label>
  <label><input type="checkbox" id="chkMark"/> 标记</label>
  <span id="markTools">
    <span>名称</span><input id="nameIn" type="text" placeholder="如 起弧"/>
    <button type="button" id="btnOk">确定</button>
    <button type="button" id="btnCancel">取消</button>
  </span>
  <button type="button" id="btnSaveHtml">保存HTML</button>
  <span id="markMsg"></span>
</div>
<div id="wrap">
  <canvas id="c"></canvas>
  <div id="side">
    <div id="hud">移动鼠标查看最近点</div>
    <h3>范围 / 起终点</h3>
    <div id="stat"></div>
    <h3>点表</h3>
    <table><thead><tr><th>#</th><th>t</th><th>X</th><th>Y</th><th>Z</th></tr></thead><tbody id="tb"></tbody></table>
  </div>
</div>
<script type="application/json" id="savedState">__STATE__</script>
<script>
var pts = __POINTS__;
var savedState = {};
try { savedState = JSON.parse(document.getElementById("savedState").textContent || "{}"); } catch (e) { savedState = {}; }
var canvas = document.getElementById("c");
var ctx = canvas.getContext("2d");
var yaw = savedState.yaw != null ? savedState.yaw : 0.75;
var pitch = savedState.pitch != null ? savedState.pitch : 0.5;
var dist = savedState.dist != null ? savedState.dist : 1;
var drag = false, lx = 0, ly = 0, hover = -1, picked = -1, downX = 0, downY = 0;
var marks = savedState.marks || [];
var PAL = ["#29b6f6","#ab47bc","#ff7043","#26a69a","#ec407a","#7e57c2","#66bb6a","#ffa726"];
var chkAxes = document.getElementById("chkAxes");
var chkData = document.getElementById("chkData");
var chkMark = document.getElementById("chkMark");
var markTools = document.getElementById("markTools");
var side = document.getElementById("side");
if (savedState.axes) chkAxes.checked = true;
if (savedState.data) chkData.checked = true;
if (savedState.marking) chkMark.checked = true;
function n(v){ return (v==null||isNaN(v)) ? "—" : Number(v).toFixed(2); }
function bounds(includeOrigin){
  if(!pts.length) return {c:[0,0,0], r:1, min:[0,0,0], max:[0,0,0]};
  var min=[pts[0].x, pts[0].y, pts[0].z], max=[pts[0].x, pts[0].y, pts[0].z];
  for (var i=1;i<pts.length;i++){
    var p=[pts[i].x, pts[i].y, pts[i].z];
    for (var k=0;k<3;k++){ if(p[k]<min[k])min[k]=p[k]; if(p[k]>max[k])max[k]=p[k]; }
  }
  if (includeOrigin){
    for (var k=0;k<3;k++){ if(0<min[k])min[k]=0; if(0>max[k])max[k]=0; }
  }
  var c=[(min[0]+max[0])/2,(min[1]+max[1])/2,(min[2]+max[2])/2];
  var r=1;
  for (var i=0;i<3;i++){
    r=Math.max(r, Math.abs(min[i]-c[i]), Math.abs(max[i]-c[i]));
  }
  return {c:c, r:r, min:min, max:max};
}
var B = bounds(false);
function refreshBounds(){ B = bounds(!!chkAxes.checked); }
function niceStep(span){
  var raw = Math.max(span, 1) / 5, mag = Math.pow(10, Math.floor(Math.log10(raw))), nrm = raw/mag;
  return nrm<1.5?mag:(nrm<3.5?2*mag:(nrm<7.5?5*mag:10*mag));
}
function project(x,y,z){
  x-=B.c[0]; y-=B.c[1]; z-=B.c[2];
  var cy=Math.cos(yaw), sy=Math.sin(yaw), cp=Math.cos(pitch), sp=Math.sin(pitch);
  var x1=x*cy - y*sy, y1=x*sy + y*cy;
  var y2=y1*cp - z*sp, z2=y1*sp + z*cp;
  var s = Math.min(canvas.width, canvas.height) * 0.86 / (B.r * dist * 2);
  return [canvas.width/2 + x1*s, canvas.height/2 - z2*s];
}
function drawAxes(){
  var step = niceStep(B.r*2);
  var reach = Math.ceil((B.r*1.15)/step)*step;
  var cols=["#e53935","#43a047","#1e88e5"], names=["X","Y","Z"];
  ctx.font="12px Consolas,Microsoft YaHei";
  for (var a=0;a<3;a++){
    var p0=[0,0,0], p1=[0,0,0]; p1[a]=reach;
    var n1=[0,0,0]; n1[a]=-reach;
    var A=project(n1[0],n1[1],n1[2]), C=project(p1[0],p1[1],p1[2]);
    ctx.strokeStyle=cols[a]; ctx.lineWidth=1.8;
    ctx.beginPath(); ctx.moveTo(A[0],A[1]); ctx.lineTo(C[0],C[1]); ctx.stroke();
    ctx.fillStyle=cols[a];
    ctx.fillText(names[a]+" mm", C[0]+6, C[1]-4);
    ctx.lineWidth=1;
    for (var v=-reach; v<=reach+1e-6; v+=step){
      if (Math.abs(v)<1e-9) continue;
      var t0=[0,0,0], t1=[0,0,0]; t0[a]=v; t1[a]=v;
      var b=(a+1)%3; t0[b]=-step*0.08; t1[b]=step*0.08;
      var u=project(t0[0],t0[1],t0[2]), w=project(t1[0],t1[1],t1[2]);
      ctx.beginPath(); ctx.moveTo(u[0],u[1]); ctx.lineTo(w[0],w[1]); ctx.stroke();
      var lab=project(t1[0],t1[1],t1[2]);
      ctx.fillText(String(Math.round(v)), lab[0]+3, lab[1]+3);
    }
  }
  var o=project(0,0,0);
  ctx.fillStyle="#fff"; ctx.beginPath(); ctx.arc(o[0],o[1],3.5,0,6.28); ctx.fill();
  ctx.fillText("O(0,0,0)", o[0]+8, o[1]-8);
}
function drawPath(color, width){
  ctx.strokeStyle=color; ctx.lineWidth=width;
  ctx.beginPath();
  for (var i=0;i<pts.length;i++){
    var p=project(pts[i].x, pts[i].y, pts[i].z);
    if(i===0) ctx.moveTo(p[0],p[1]); else ctx.lineTo(p[0],p[1]);
  }
  ctx.stroke();
}
function draw(){
  refreshBounds();
  ctx.fillStyle="#111"; ctx.fillRect(0,0,canvas.width,canvas.height);
  if (chkAxes.checked) drawAxes();
  if(!pts.length){ ctx.fillStyle="#888"; ctx.fillText("无笛卡尔点", 20, 40); return; }
  drawPath("#ffcc02", 1.8);
  var s=project(pts[0].x,pts[0].y,pts[0].z), e=project(pts[pts.length-1].x,pts[pts.length-1].y,pts[pts.length-1].z);
  ctx.fillStyle="#4caf50"; ctx.beginPath(); ctx.arc(s[0],s[1],5,0,6.28); ctx.fill();
  ctx.fillStyle="#ff5252"; ctx.beginPath(); ctx.arc(e[0],e[1],5,0,6.28); ctx.fill();
  ctx.font="12px Microsoft YaHei,Consolas";
  for (var m=0;m<marks.length;m++){
    var mp=project(marks[m].x, marks[m].y, marks[m].z);
    var col=PAL[m % PAL.length];
    ctx.globalAlpha=0.35; ctx.fillStyle=col;
    ctx.beginPath(); ctx.arc(mp[0],mp[1],16,0,6.28); ctx.fill();
    ctx.globalAlpha=1; ctx.beginPath(); ctx.arc(mp[0],mp[1],8,0,6.28); ctx.fill();
    ctx.strokeStyle="#fff"; ctx.lineWidth=1.6; ctx.stroke();
    ctx.fillStyle="#fff";
    ctx.fillText(marks[m].name, mp[0]+12, mp[1]-8);
  }
  if (picked>=0){
    var q=project(pts[picked].x, pts[picked].y, pts[picked].z);
    ctx.strokeStyle="#fff"; ctx.lineWidth=2; ctx.setLineDash([5,4]);
    ctx.beginPath(); ctx.arc(q[0],q[1],12,0,6.28); ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(q[0]-16,q[1]); ctx.lineTo(q[0]+16,q[1]);
    ctx.moveTo(q[0],q[1]-16); ctx.lineTo(q[0],q[1]+16); ctx.stroke();
  }
  if (hover>=0 && hover!==picked){
    var h=project(pts[hover].x, pts[hover].y, pts[hover].z);
    ctx.strokeStyle="#fff"; ctx.beginPath(); ctx.arc(h[0],h[1],7,0,6.28); ctx.stroke();
  }
  var look = picked>=0 ? picked : hover;
  if (look>=0){
    var lp=pts[look];
    var tip="X="+n(lp.x)+"  Y="+n(lp.y)+"  Z="+n(lp.z)+" mm   Rx="+n(lp.rx)+"  Ry="+n(lp.ry)+"  Rz="+n(lp.rz)+" °";
    ctx.font="13px Consolas,Microsoft YaHei";
    var tw=ctx.measureText(tip).width+20;
    ctx.fillStyle="rgba(0,0,0,0.7)";
    ctx.fillRect(10, canvas.height-34, tw, 24);
    ctx.fillStyle="#ffcc02";
    ctx.fillText(tip, 20, canvas.height-17);
  }
}
function markIndexOf(i){
  for (var k=0;k<marks.length;k++){ if(marks[k].i===i) return k; }
  return -1;
}
function setPicked(i){
  picked=i;
  if(i<0){
    document.getElementById("markMsg").textContent="先单击选点预览，再确定标记";
    draw();
    return;
  }
  var p=pts[i], mi=markIndexOf(i);
  if(mi>=0) document.getElementById("nameIn").value=marks[mi].name;
  document.getElementById("markMsg").textContent="预览  X="+n(p.x)+"  Y="+n(p.y)+"  Z="+n(p.z)+" mm"+(mi>=0?"  「"+marks[mi].name+"」":"");
  draw();
}
function confirmMark(){
  if(!chkMark.checked) return;
  if(picked<0){
    document.getElementById("markMsg").textContent="还没有选点";
    return;
  }
  var name=(document.getElementById("nameIn").value||"").replace(/^\s+|\s+$/g,"");
  if(!name){
    document.getElementById("markMsg").textContent="先填写该点的实际意义";
    return;
  }
  var p=pts[picked], found=markIndexOf(picked);
  var rec={name:name, i:picked, t:p.t, x:p.x, y:p.y, z:p.z, rx:p.rx, ry:p.ry, rz:p.rz, j:p.j.slice()};
  if(found>=0) marks[found]=rec; else marks.push(rec);
  document.getElementById("markMsg").textContent="已确定「"+name+"」";
  picked=-1;
  draw();
}
function cancelMark(){
  if(!chkMark.checked) return;
  if(picked<0){
    document.getElementById("markMsg").textContent="没有可取消的选择";
    return;
  }
  var found=markIndexOf(picked);
  if(found>=0){
    var name=marks[found].name;
    marks.splice(found,1);
    document.getElementById("markMsg").textContent="已取消标记「"+name+"」";
  } else {
    document.getElementById("markMsg").textContent="已取消选点";
  }
  picked=-1;
  draw();
}
function hitMark(mx,my){
  var best=-1, bd=18*18;
  for (var m=0;m<marks.length;m++){
    var p=project(marks[m].x, marks[m].y, marks[m].z);
    var d=(p[0]-mx)*(p[0]-mx)+(p[1]-my)*(p[1]-my);
    if(d<bd){ bd=d; best=marks[m].i; }
  }
  return best;
}
function saveHtml(){
  var st={marks:marks, yaw:yaw, pitch:pitch, dist:dist, axes:!!chkAxes.checked, data:!!chkData.checked, marking:!!chkMark.checked};
  document.getElementById("savedState").textContent=JSON.stringify(st);
  if (chkAxes.checked) chkAxes.setAttribute("checked","checked"); else chkAxes.removeAttribute("checked");
  if (chkData.checked) chkData.setAttribute("checked","checked"); else chkData.removeAttribute("checked");
  if (chkMark.checked) chkMark.setAttribute("checked","checked"); else chkMark.removeAttribute("checked");
  var html="<!DOCTYPE html>\n"+document.documentElement.outerHTML;
  var blob=new Blob([html], {type:"text/html;charset=utf-8"});
  var a=document.createElement("a");
  a.href=URL.createObjectURL(blob);
  a.download=(document.title||"traj")+"_marked.html";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(a.href);
  document.getElementById("markMsg").textContent="已保存当前HTML";
}
function showHover(i){
  hover=i;
  var el=document.getElementById("hud");
  if(i<0){ el.textContent="移动鼠标查看最近点"; draw(); return; }
  var p=pts[i];
  el.textContent="点 "+(i+1)+" / "+pts.length+"   t="+n(p.t)+" s\n"+
    "笛卡尔  X="+n(p.x)+"  Y="+n(p.y)+"  Z="+n(p.z)+" mm\n"+
    "姿态    Rx="+n(p.rx)+"  Ry="+n(p.ry)+"  Rz="+n(p.rz)+" °\n"+
    "关节    "+p.j.map(function(v,k){return "J"+(k+1)+"="+n(v);}).join("  ")+" °";
  var rows=document.querySelectorAll("#tb tr");
  for (var r=0;r<rows.length;r++) rows[r].className = (r===i ? "sel" : "");
  if (chkData.checked && rows[i]) rows[i].scrollIntoView({block:"nearest", inline:"nearest"});
  draw();
}
function nearest(mx,my){
  var best=-1, bd=64;
  for (var i=0;i<pts.length;i++){
    var p=project(pts[i].x, pts[i].y, pts[i].z);
    var d=(p[0]-mx)*(p[0]-mx)+(p[1]-my)*(p[1]-my);
    if(d<bd){ bd=d; best=i; }
  }
  return best;
}
function fillStats(){
  if(!pts.length){ document.getElementById("stat").textContent="无数据"; return; }
  var a=pts[0], b=pts[pts.length-1], len=0;
  for (var i=1;i<pts.length;i++){
    var dx=pts[i].x-pts[i-1].x, dy=pts[i].y-pts[i-1].y, dz=pts[i].z-pts[i-1].z;
    len+=Math.sqrt(dx*dx+dy*dy+dz*dz);
  }
  document.getElementById("stat").innerHTML =
    "起点  X "+n(a.x)+"  Y "+n(a.y)+"  Z "+n(a.z)+"<br>"+
    "终点  X "+n(b.x)+"  Y "+n(b.y)+"  Z "+n(b.z)+"<br>"+
    "最小  X "+n(B.min[0])+"  Y "+n(B.min[1])+"  Z "+n(B.min[2])+"<br>"+
    "最大  X "+n(B.max[0])+"  Y "+n(B.max[1])+"  Z "+n(B.max[2])+"<br>"+
    "路径长 "+n(len)+" mm";
  var tb=document.getElementById("tb"); tb.innerHTML="";
  for (var i=0;i<pts.length;i++){
    var tr=document.createElement("tr");
    tr.innerHTML="<td>"+(i+1)+"</td><td>"+n(pts[i].t)+"</td><td>"+n(pts[i].x)+"</td><td>"+n(pts[i].y)+"</td><td>"+n(pts[i].z)+"</td>";
    (function(idx){ tr.onmouseover=function(){ showHover(idx); }; })(i);
    tb.appendChild(tr);
  }
}
function resize(){
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  draw();
}
function syncMarkMode(){
  markTools.style.display = chkMark.checked ? "inline-flex" : "none";
  if (!chkMark.checked){
    picked=-1;
    document.getElementById("markMsg").textContent="";
  } else {
    document.getElementById("markMsg").textContent="单击选点预览，确定后才标记";
  }
  draw();
}
chkAxes.addEventListener("change", draw);
chkData.addEventListener("change", function(){ side.className = chkData.checked ? "" : "hidden"; resize(); });
document.getElementById("btnSaveHtml").addEventListener("click", saveHtml);
document.getElementById("btnOk").addEventListener("click", confirmMark);
document.getElementById("btnCancel").addEventListener("click", cancelMark);
document.getElementById("nameIn").addEventListener("keydown", function(e){ if(e.key==="Enter") confirmMark(); });
side.className = savedState.data ? "" : "hidden";
canvas.addEventListener("mousedown", function(e){
  drag=true; lx=e.clientX; ly=e.clientY; downX=e.clientX; downY=e.clientY;
  canvas.style.cursor="grabbing";
});
window.addEventListener("mouseup", function(e){
  if (drag && chkMark.checked){
    var dx=e.clientX-downX, dy=e.clientY-downY;
    if (dx*dx+dy*dy < 16){
      var rec=canvas.getBoundingClientRect();
      var mx=e.clientX-rec.left, my=e.clientY-rec.top;
      var hit=hitMark(mx,my);
      if(hit<0) hit=nearest(mx,my);
      setPicked(hit);
    }
  }
  drag=false;
  canvas.style.cursor="grab";
});
window.addEventListener("mousemove", function(e){
  if(drag){
    var moved=(e.clientX-downX)*(e.clientX-downX)+(e.clientY-downY)*(e.clientY-downY);
    if (moved < 16) return;
    yaw += (e.clientX-lx)*0.008; pitch += (e.clientY-ly)*0.008;
    if(pitch>1.4) pitch=1.4; if(pitch<-1.4) pitch=-1.4;
    lx=e.clientX; ly=e.clientY; draw();
    return;
  }
  var r=canvas.getBoundingClientRect();
  if(e.clientX>=r.left && e.clientX<=r.right && e.clientY>=r.top && e.clientY<=r.bottom){
    showHover(nearest(e.clientX-r.left, e.clientY-r.top));
  }
});
canvas.addEventListener("wheel", function(e){
  e.preventDefault();
  dist *= (e.deltaY>0 ? 1.08 : 0.92);
  if(dist<0.2) dist=0.2; if(dist>8) dist=8; draw();
}, {passive:false});
window.addEventListener("blur", function(){ drag=false; canvas.style.cursor="grab"; });
syncMarkMode();
chkMark.addEventListener("change", syncMarkMode);
fillStats();
resize();
</script>
</body>
</html>
"""

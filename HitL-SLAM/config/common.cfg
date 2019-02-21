-- definitions common to all config files

pi  = math.pi;
off = false;
on  = true;

-- helpers

abs = math.abs;
sin = math.sin;
cos = math.cos;

function sq(x)
   return x*x
end

function circle_area(rad)
   return pi * rad * rad
end

function deg2rad(a)
   return a * pi / 180.0;
end

function iff(sel,a,b)
   if(sel) then
      return a;
   else
      return b;
   end
end

-- constructors

function vec2(_x,_y)
  return {x=_x,y=_y}
end

function vec3(_x,_y,_z)
  return {x=_x,y=_y,z=_z}
end

function quat4(_w,_x,_y,_z)
  return {w=_w,x=_x,y=_y,z=_z}
end

function range(_min,_max)
  return {min=_min,max=_max}
end

function range_empty(_min_max)
  return {min=_min_max,max=_min_max}
end

function bbox2d(cx,cy,rx,ry)
  return {
     cen = vec2(cx,cy);
     rad = vec2(rx,ry);
  }
end

function bbox2d_xxyy(x0,x1,y0,y1)
  return {
     cen = vec2(    (x1+x0)/2 ,    (y1+y0)/2 );
     rad = vec2(abs((x1-x0)/2),abs((y1-y0)/2));
  }
end

function bbox2d_xxcr(x0,x1,cy,ry)
  return {
     cen = vec2(    (x1+x0)/2 ,cy);
     rad = vec2(abs((x1-x0)/2),ry);
  }
end

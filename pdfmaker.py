from reportlab.pdfgen.canvas import Canvas
from math import atan2, pi
from urllib.request import urlretrieve
from tempfile import TemporaryDirectory
from uuid import uuid5, NAMESPACE_URL
from os.path import exists
import sys, shutil
from PIL import Image

from reportlab.pdfgen.pathobject import PDFPathObject as Path

viewport = (792,445.5)

javascript_code_to_revist = """
(c, r, p0, p3) => {
    // assumes p3 has greater angle (by less than PI) than 0; if not, swap sign of m
    // looks good up to just over pi/2 radians; more starts looking bad
    let v1 = [c[1]-p0[1], p0[0]-c[0]];
    let v2 = [p3[1]-c[1], c[0]-p3[0]];
    let d = (p0[0]-c[0])*(p3[0]-c[0]) + (p0[1]-c[1])*(p3[1]-c[1]);
    let cs = d / (r**2);
    let t = Math.sqrt(1-cs)/(Math.SQRT2 + Math.sqrt(1+cs));
    let m = 4*t/3;
    return [p0,
        [p0[0]+v1[0]*m, p0[1]+v1[1]*m],
        [p3[0]+v2[0]*m, p3[1]+v2[1]*m],
    p3];
}

// cubic bezier approximation of arc from pA to pB (both assumed to be on edge of circle) around circle with center c and radius r. Uses ceil(angle/90°) curves and has up to 0.027% error. Uses two sqrt and no other library functions.
function bezchain(c, r, pA, pB) {
    const p0 = [pA[0]-c[0], pA[1]-c[1]];
    const p1 = [pB[0]-c[0], pB[1]-c[1]];
    const p90 = [ // every 90° rotation of p0 in increasing-angle order
        [ p0[0], p0[1]],
        [-p0[1], p0[0]],
        [-p0[0],-p0[1]],
        [ p0[1],-p0[0]],
    ];
    const c90 = p90.map(p => p[0]*p1[1]-p[1]*p1[0]); // p90[i] cross p1
    console.log(c90);
    const ans = [];
    let m = 4/(3*(Math.SQRT2+1));
    // find p90 quadrant of c90
    let bits = 0;
    while((c90[bits&3] < -1e-4 || c90[(bits+1)&3] > 1e-4) && bits < 10) {
        ans.push([ // a 90° segment
            [c[0]+p90[bits][0], c[1]+p90[bits][1]],
            [c[0]+p90[bits][0] - m*p90[bits][1], c[1]+p90[bits][1] + m*p90[bits][0]],
            [c[0]+p90[bits+1][0] + m*p90[bits+1][1], c[1]+p90[bits+1][1] - m*p90[bits+1][0]],
            [c[0]+p90[bits+1][0], c[1]+p90[bits+1][1]],
        ]);
        bits += 1;
    }
    let dot = p90[bits][0]*p1[0] + p90[bits][1]*p1[1];
    let cosine = dot / r**2;
    m = 4*Math.sqrt(1-cosine)/(3*(Math.SQRT2 + Math.sqrt(1+cosine)));
    ans.push([ // a 90° segment
        [c[0]+p90[bits][0], c[1]+p90[bits][1]],
        [c[0]+p90[bits][0] - m*p90[bits][1], c[1]+p90[bits][1] + m*p90[bits][0]],
        [c[0]+p1[0] + m*p1[1], c[1]+p1[1] - m*p1[0]],
        [c[0]+p1[0], c[1]+p1[1]],
    ]);
    return ans;
}

"""

def archelp(path:Path, xyr:list[float], radians1:float, radians2:float) -> None:
    """Adds an arc to the given path.
    
    path - the path object to add an arc to
    xyr - the [x,y,radius] circle to have the arc go around
    radians1 - the starting angle, in radius
    radians2 - the ending angle, in radius
    
    Always goes in increasing-angle direction, even if radians2 < radians1.
    """
    
    if xyr[2] == 0: return
    if radians2 < radians1: radians2 += 2*pi
    path.arcTo(
        xyr[0]-xyr[2], xyr[1]-xyr[2], 
        xyr[0]+xyr[2], xyr[1]+xyr[2],
        radians1*180/pi,
        (radians2-radians1)*180/pi,
    )
    """Aside: there is no arc literal in PDF, and it does make stroke merging more complicated.
    An alternative: a common (though not quite optimal) circular arc Bézier curve approximation uses:
    
    - P0 and P3 = endpoints of arc
    - P1 and P3 are on tangents; i.e. for P1, perp of P0-C
    - Scale of P1/P3 is 4/3 tan(angle/4)
    
    If there's at least 4 curves for a complete circle, this is visually quite good,
    suggesting one way to cap ends would be to put a point in the middle of the end cap, 
    approximating the cap with 2 curves.
    
    
    aside: tan(t/4) = sqrt(1-cos(t))/(sqrt(2)+sqrt(1+cos(t))) for -pi<=t<=pi
        and cos(t) can be found with dot products and a sqrt,
        but it's unclear to me if that saves much over two atan2 and a tan
    """

    

def tdot(p:list[float], c:Canvas) -> Path:
    """Returns a path which is just one circle.
    
    p - the [x,y,r] circle
    c - a Canvas whose beginPath method will be used
    """
    ans = c.beginPath()
    ans.circle(p[0], p[1], p[2])
    return ans

def tpill(p,q, c):
    H = [a-b for a,b in zip(p,q)]
    ell = H[0]**2 + H[1]**2
    if ell <= H[2]**2:
        return tdot(p if p[2] > q[2] else q, c)
    sm = (ell - H[2]**2)**0.5/ell
    bm = -H[2]/ell
    
    pb = (H[0] * p[2] * bm, H[1] * p[2] * bm)
    ps = (-H[1] * p[2] * sm, H[0] * p[2] * sm)
    
    qb = (H[0] * q[2] * bm, H[1] * q[2] * bm)
    qs = (-H[1] * q[2] * sm, H[0] * q[2] * sm)
    
    trap = (
        (p[0]+pb[0]+ps[0], p[1]+pb[1]+ps[1]),
        (q[0]+qb[0]+qs[0], q[1]+qb[1]+qs[1]),
        (q[0]+qb[0]-qs[0], q[1]+qb[1]-qs[1]),
        (p[0]+pb[0]-ps[0], p[1]+pb[1]-ps[1]),
    )
    
    ans = c.beginPath()
    ans.moveTo(*trap[0])
    ans.lineTo(*trap[1])
    archelp(ans, q, atan2(trap[1][1]-q[1], trap[1][0]-q[0]), atan2(trap[2][1]-q[1], trap[2][0]-q[0]))
    ans.lineTo(*trap[3])
    archelp(ans, p, atan2(trap[3][1]-p[1], trap[3][0]-p[0]), atan2(trap[0][1]-p[1], trap[0][0]-p[0]))
    ans.close()
    return ans


#################
def fix_negative_radius(pts):
    for i in range(len(pts)):
        if isinstance(pts[i],list):
            pts[i][2] = max(0, pts[i][2])
        else:
            pts[i] = (*pts[i][:2], max(0, pts[i][2]))
    # for pt in pts:
        # pts[2] = max(0, pt[2])

add = lambda *args: tuple(sum(arg) for arg in zip(*args))
sub = lambda x,y: tuple(a-b for a,b in zip(x,y))
mul = lambda x,s: tuple(e*s for e in x)
dot = lambda x,y: sum(a*b for a,b in zip(x,y))
r90 = lambda p: (-p[1],p[0])

def mirror_along(pt, s_near, s_far):
    a = sub(pt,s_near);
    b = sub(s_far, s_near);
    den = dot(b,b)
    if den == 0: return pt
    return add(s_far, add(a,mul(b, -2*dot(a, b)/den)));


def ccr2bez(p0,p1,p2,p3):
    if not p0: p0 = mirror_along(p3,p2,p1);
    if not p3: p3 = mirror_along(p0,p1,p2);

    l0 = ((p0[0]-p1[0])**2 + (p0[1]-p1[1])**2)**0.25
    l1 = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.25
    l2 = ((p2[0]-p3[0])**2 + (p2[1]-p3[1])**2)**0.25
    if (l1 == 0): return (p1,p1,p2,p2)
    i0,i2 = l0/l1, l2/l1
    i01, i21 = 1+i0, 1+i2
    
    return [p1,
        p1 if i0==0 else add(mul(p0,-1/(3*i0*i01)), mul(p1,(i0+i01)/(3*i0)), mul(p2,i0/(3*i01))),
        p2 if i2==0 else add(mul(p3,-1/(3*i2*i21)), mul(p2,(i2+i21)/(3*i2)), mul(p1,i2/(3*i21))),
        p2]

def minimize_strain_hermite(p0,v0,v1,p1):
    p10 = sub(p1,p0)
    v00, v11, v01, pv0, pv1 = dot(v0,v0), dot(v1,v1), dot(v0,v1), dot(p10,v0), dot(p10,v1)
    if not v00 or not v11: return 1,1
    a0 = (6*pv0*v11 - 3*pv1*v01) / (4*v00*v11 - v01**2)
    a1 = (6*pv1*v00 - 3*pv0*v01) / (4*v00*v11 - v01**2)
    return (a0,a1)

def bendsides(s0,s1,s2,s3):
    xyr = ccr2bez(s0,s1,s2,s3)
    fix_negative_radius(xyr)
    n = len(xyr)-1
    # points
    H = (sub(xyr[1],xyr[0]), sub(xyr[n], xyr[n-1]))
    a = tuple(e[:2] for e in H)
    ell = tuple(dot(e,e) for e in a)
    Hr2 = tuple(e[2]**2 for e in H)
    if any(e==0 or e<r for e,r in zip(ell, Hr2)): return False,False
    p = (xyr[0][:2], xyr[n][:2])
    r = (xyr[0][2], xyr[n][2])
    s = tuple((-e[1], e[0]) for e in a)
    am = tuple(-e[2]/l for e,l in zip(H, ell))
    sm = tuple((e - r)**0.5/e for e,r in zip(ell, Hr2))

    # build the result
    p_a = tuple(add(e, mul(a_, am_*r_)) for e,a_,am_,r_ in zip(p,a,am,r))
    ss = tuple(mul(e, sm_*r_) for e,sm_,r_ in zip(s,sm,r))
    pts = tuple((add(e,s_),sub(e,s_)) for e,s_ in zip(p_a,ss))
    
    # non-normalized derivatives: perpendicular of vector to circle centers
    deriv = tuple([r90(sub(pt,p_)) for pt in end] for end,p_ in zip(pts,p))
    for side in range(2):
        a0,a1 = minimize_strain_hermite(pts[0][side], deriv[0][side], deriv[1][side], pts[1][side])
        deriv[0][side] = mul(deriv[0][side], a0/3)
        deriv[1][side] = mul(deriv[1][side], a1/3)

    # path creation
    R = (pts[0][0], add(pts[0][0], deriv[0][0]), sub(pts[1][0],deriv[1][0]), pts[1][0])
    L = (pts[0][1], add(pts[0][1], deriv[0][1]), sub(pts[1][1],deriv[1][1]), pts[1][1])
    return R,L
    
def tbend(s0,s1,s2,s3, c):
    '''Unused'''
    R,L = bendsides(s0,s1,s2,s3)
    if not R or not L:
        return tpill(s1,s2, c)

    ans = c.beginPath()
    ans.moveTo(
        *L[0],
    )
    ans.curveTo(
        *L[1],
        *L[2],
        *L[3],
    )
    archelp(ans, s2,
        atan2(L[3][1]-s2[1], L[3][0]-s2[0]), 
        atan2(R[3][1]-s2[1], R[3][0]-s2[0]),
    )
    ans.curveTo(
        *R[2],
        *R[1],
        *R[0],
    )
    archelp(ans, s1, 
        atan2(R[0][1]-s1[1], R[0][0]-s1[0]),
        atan2(L[0][1]-s1[1], L[0][0]-s1[0]),
    )
    ans.close()
    return ans

def pillify(p0, Rs, Ls, pn, c):
    ans = c.beginPath()
    ans.moveTo(
        *Ls[0][0],
    )
    for L in Ls:
        ans.curveTo(
            *L[1],
            *L[2],
            *L[3],
        )
    archelp(ans, pn,
        atan2(Ls[-1][-1][1]-pn[1], Ls[-1][-1][0]-pn[0]), 
        atan2(Rs[-1][-1][1]-pn[1], Rs[-1][-1][0]-pn[0]),
    )
    for R in reversed(Rs):
        ans.curveTo(
            *R[2],
            *R[1],
            *R[0],
        )
    archelp(ans, p0, 
        atan2(Rs[0][0][1]-p0[1], Rs[0][0][0]-p0[0]),
        atan2(Ls[0][0][1]-p0[1], Ls[0][0][0]-p0[0]),
    )
    ans.close()
    return ans
    

def tstroke(p, c):
    if len(p) == 1:
        return [tdot(p[0], c)]
    elif len(p) == 2:
        return [tpill(p[0], p[1], c)]
    else:
        ans = []
        r,l = [], []
        p0, pn = None, None
        for i in range(0, len(p)-1):
            R,L = bendsides(p[i-1] if i>0 else None, p[i], p[i+1], p[i+2] if i+2 < len(p) else None)
            if not R or not L:
                if len(r) > 0:
                    ans.append(pillify(p0, r, l, pn, c))
                    r,l = [],[]
                    p0,pn = None,None
                ans.append(tpill(p[i], p[i+1], c))
            else:
                r.append(R)
                l.append(L)
                pn = p[i+1]
                if p0 is None: p0 = p[i]
        if len(r) > 0:
            ans.append(pillify(p0, r, l, pn, c))
        return ans




def logreader(fname):
    import json
    before = []
    after = []
    live = {'zoom':720, 'dx':0, 'dy':0, 'strokes':[]}
    for line in open(fname):
        a = json.loads(line)
        args = a.get('args',[])
        if a['mode'] == 'draw': live['strokes'].append(a)
        elif a['mode'] == 'erase': live['strokes'].append(a)
        elif a['mode'] == 'image': live['strokes'].append(a)
        elif a['mode'] == 'pan':
            if args[0] == 'reset':
                live['dx'] += 0
                live['dy'] += 0
            else:
                live['dx'] += args[0]
                live['dy'] += args[1]
        elif a['mode'] == 'zoom':
            if args[0] == 'reset':
                live['zoom'] = 720
            else:
                live['zoom'] *= args[0]
        elif a['mode'] == 'clone':
            nslide = {'zoom':live['zoom'], 'dx':live['dx'], 'dy':live['dy'], 'strokes':live['strokes'][:]}
            if args[0]  == 'back': after.append(nslide)
            else: before.append(nslide)
        elif a['mode'] == 'insert':
            if args[0] == 'back': after.append(live)
            else: before.append(live)
            live = {'zoom':720, 'dx':0, 'dy':0, 'strokes':[]}
        elif a['mode'] == 'go':
            if args[0] == 'forward':
                before.append(live)
                live = after.pop()
            if args[0] == 'back':
                after.append(live)
                live = before.pop()
            if args[0] == 'end':
                while len(after) > 0:
                    before.append(live)
                    live = after.pop()
            if args[0] == 'start':
                while len(before) > 0:
                    before.append(live)
                    live = after.pop()
    return before + [live] + list(reversed(after))

def hexcolor2tuple(hc):
    hc = hc[1:] # remove hashtag
    nybles = len(hc)//3
    return tuple(int(hc[i*nybles:(i+1)*nybles],16)/((16<<(nybles-1))-1) for i in range(3))

def log2pdf(logname, pdfname):
    slides = logreader(logname)
    
    c = Canvas(pdfname, pagesize=viewport, bottomup=0, pageCompression=0)
    
    imgs = {}
    with TemporaryDirectory() as idir:
        for slide in slides:
            c.setFillColorRGB(1,1,1)
            c.rect(0, 0, viewport[0], viewport[1], stroke=0, fill=1)
            
            c.translate(viewport[0]/2, viewport[1]/2)
            
            c.scale(viewport[1]/slide['zoom'], viewport[1]/slide['zoom'])
            c.translate(slide['dx'], slide['dy'])
            
            for stroke in slide['strokes']:
                if stroke['mode'] == 'image':
                    fn = idir+'/'+str(uuid5(NAMESPACE_URL, stroke['url']))
                    if not exists(fn):
                        if ':' in stroke['url']:
                            urlretrieve(stroke['url'], filename=fn)
                        elif exists(sys.path[0]+'/web/'+stroke['url']):
                            shutil.copy(sys.path[0]+'/web/'+stroke['url'], fn)
                        elif exists(stroke['url']):
                            shutil.copy(stroke['url'], fn)
                        else:
                            print('WARNING: missing image', stroke['url'])
                            continue
                        imgs[fn] = Image.open(fn)
                    img = imgs[fn]
                    w,h = img.size
                    c.scale(1,-1)
                    c.drawImage(fn, stroke['x'], -stroke['h']-stroke['y'], width=stroke['w'], height=stroke['h'])
                    c.scale(1,-1)
                    continue
                if stroke['mode'] == 'erase': c.setFillColorRGB(1,1,1)
                else: c.setFillColorRGB(*hexcolor2tuple(stroke['color']))
                p = stroke['points']
                mode = {'stroke':0, 'fill':1, 'fillMode':1}
                if len(p) == 1:
                    c.drawPath(tdot(p[0], c), **mode)
                elif len(p) == 2:
                    c.drawPath(tpill(p[0], p[1], c), **mode)
                else:
                    for bit in tstroke(p,c):
                        c.drawPath(bit, **mode)
            
            c.showPage()
        
        c.save()


if __name__ == '__main__':
    for arg in sys.argv[1:]:
        log2pdf(arg, arg+'.pdf')

    if len(sys.argv) <= 1:
        print('USAGE:', sys.argv[0], 'logfile [logfile*]')
        print('    saves each to logfile.pdf')
        print('    uses same rules as live display, but with tbend parts merged')
        print('    TO DO: use more time-consuming search for optimal display')

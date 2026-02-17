var def_keys = '`'+` color #000
1 color #f00
2 color #33f
3 color #080
4 color #80f
5 color #840
6 color #f90
7 color #9f0
8 color #fae
9 color #ee0
0 color #fff
t toggle_hud
i invert_colors
b softness 1
n softness 2
m softness 0.5
p putimage
- pensize 0.7937005259840998
= pensize 1.2599210498948732
_ zoom 1.2599210498948732
+ zoom 0.7937005259840998
Ctrl+Home pensize reset
Ctrl+Home zoom reset
Ctrl+Home pan reset
Home zoom reset
Home pan reset
Alt+PageUp clone back
Shift+PageUp insert back
PageUp go back
Alt+PageDown clone forward
Shift+PageDown insert forward
PageDown go forward
`

/** Returns a circular Path2D */
function tdot(p) {
    const ans = new Path2D();
    ans.arc(p[0], p[1], p[2], 0, 2*Math.PI);
    ans.closePath();
    return ans;
}

/** Returns a straight-edged trapezoidal pill Path2D for the provided xyr endpoints */
function tpill(p,q) {
    const H = p.map((e,i) => e-q[i])
    const ell = H[0]**2 + H[1]**2;
    if (ell <= H[2]**2) {
        if (p[2] >= q[2]) return tdot(p);
        else return tdot(q);
    }
    const sm = (ell - H[2]**2)**0.5/ell;
    const bm = -H[2]/ell;
    
    const pb = [H[0] * p[2] * bm, H[1] * p[2] * bm];
    const ps = [-H[1] * p[2] * sm, H[0] * p[2] * sm];
    
    const qb = [H[0] * q[2] * bm, H[1] * q[2] * bm];
    const qs = [-H[1] * q[2] * sm, H[0] * q[2] * sm];
    
    const trap = [
        [p[0]+pb[0]+ps[0], p[1]+pb[1]+ps[1]],
        [q[0]+qb[0]+qs[0], q[1]+qb[1]+qs[1]],
        [q[0]+qb[0]-qs[0], q[1]+qb[1]-qs[1]],
        [p[0]+pb[0]-ps[0], p[1]+pb[1]-ps[1]],
    ];

    const ans = new Path2D()
    ans.moveTo(...trap[0]);
    ans.lineTo(...trap[1]);
    ans.arc(
        q[0], q[1], q[2], 
        Math.atan2(trap[1][1]-q[1], trap[1][0]-q[0]), 
        Math.atan2(trap[2][1]-q[1], trap[2][0]-q[0]),
    );
    ans.lineTo(...trap[3]);
    ans.arc(
        p[0], p[1], p[2], 
        Math.atan2(trap[3][1]-p[1], trap[3][0]-p[0]), 
        Math.atan2(trap[0][1]-p[1], trap[0][0]-p[0]),
    );
    ans.closePath();
    return ans;
}


/** Given a list of xyr points, turns any negative r (possible due to spline) into 0 */
function fix_negative_radius(pts) {
    pts.forEach(pt => pt[2] = Math.max(0, pt[2]));
}

/**
 * Finds the Bezier control points of the centripetal Catmull-Rom spline through the middle two points.
 * For xyr points, uses only xy, not r, for the weights.
 * */
function ccr2bez(p0,p1,p2,p3) {
    if (!p0) p0 = mirror_along(p3,p2,p1);
    if (!p3) p3 = mirror_along(p0,p1,p2);
    
    const l0 = ((p0[0]-p1[0])**2 + (p0[1]-p1[1])**2)**0.25;
    const l1 = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.25;
    const l2 = ((p2[0]-p3[0])**2 + (p2[1]-p3[1])**2)**0.25;
    if (l1 == 0) return [p1,p1,p2,p2];
    const i0 = l0/l1, i2 = l2/l1;
    const i01 = 1+i0, i21 = 1+i2;
    
    return [p1,
    i0==0 ? p1 : [[p0,-1/(3*i0*i01)], [p1,(i0+i01)/(3*i0)], [p2,i0/(3*i01)]]
        .map(([p,s]) => p.map(e=>e*s))
        .reduce((p,q) => p.map((e,i)=>e+q[i])),
    i2 == 0 ? p2 : [[p3,-1/(3*i2*i21)], [p2,(i2+i21)/(3*i2)], [p1,i2/(3*i21)]]
        .map(([p,s]) => p.map(e=>e*s))
        .reduce((p,q) => p.map((e,i)=>e+q[i])),
    p2];
}

/// vector math
const add = (x,y) => x.map((e,i)=>e+y[i]);
const sub = (x,y) => x.map((e,i)=>e-y[i]);
const mul = (x,s) => x.map(e=>e*s);
const dot = (x,y) => x.map((e,i)=>e*y[i]).reduce((s,t)=>s+t);
/// vector linear interpolation
const lerp = (p0,p1,t) => p0.map((e,i) => (1-t)*e+t*p1[i]);

/// given (a,b,c) returns d such that (a,b,c,d) is symmetric around the hyperplane bisecting (b,c)
const mirror_along = (pt, s_near, s_far) => {
    const a = sub(pt,s_near);
    const b = sub(s_far, s_near);
    const den = dot(b,b);
    if (!den) return pt;
    return add(s_far, add(a,mul(b, -2*dot(a, b)/den)));
}

/// given a list of vector control points of a Bézier curve, split it at t and return the front and back halves' control points
const bezcut = (cp, t) => {
    const n = cp.length-1;
    const f = cp.map(e=>cp[0]), b = cp.map(e=>cp[n]);
    for(let i=1; i<=n; i+=1) {
        cp = cp.slice(1).map((e,i) => lerp(cp[i],e,t))
        f[i] = cp[0];
        b[n-i] = cp[n-i];
    }
    return [f, b];
};

/** Given a Hermite polynomial (endpoints and tanget directions), return scales of tangent directions to minimize strain */
function minimize_strain_hermite(p0,v0,v1,p1) {
    // See https://doi.org/10.1016/j.cagd.2003.08.003
    // input: cubic hermite polynomial: endpoints and tangent directions
    // output: scalars to multiply v0 and v1 by to minimize strain
    let p10 = sub(p1,p0);
    let v00 = dot(v0,v0), v11 = dot(v1,v1), v01 = dot(v0,v1), pv0 = dot(p10,v0), pv1 = dot(p10,v1);
    if (v00 === 0 || v11 === 0) return [1,1];
    let a0 = (6*pv0*v11 - 3*pv1*v01) / (4*v00*v11 - v01**2);
    let a1 = (6*pv1*v00 - 3*pv0*v01) / (4*v00*v11 - v01**2);
    return [a0,a1];
}

/** Given four sample points, return a Path2D that bridges the middle two. */
function tbend(s0,s1,s2,s3) {
    const xyr = ccr2bez(s0,s1,s2,s3);
    fix_negative_radius(xyr);
    const n = xyr.length-1;
    // points
    const H = [sub(xyr[1],xyr[0]), sub(xyr[n], xyr[n-1])]
    const a = H.map(e=>e.slice(0,2));
    const ell = a.map(e=>dot(e,e));
    const Hr2 = H.map(e=>e[2]**2);
    if (ell.map((e,i) => e==0 || e<Hr2[i]).reduce((a,b)=>a||b)) return tpill(s1,s2);
    const p = [xyr[0], xyr[n]].map(e=>e.slice(0,2));
    const r = [xyr[0], xyr[n]].map(e=>e[2]);
    const s = a.map(e=>[-e[1], e[0]]);
    const am = H.map((e,i)=>-e[2]/ell[i]);
    const sm = ell.map((e,i)=>(e - Hr2[i])**0.5/e);

    // build the result
    const p_a = p.map((e,i)=>add(e, mul(a[i], am[i]*r[i])));
    const ss = s.map((e,i)=>mul(e,  sm[i]*r[i]));
    const pts = p_a.map((e,i)=>[add(e, ss[i]), sub(e, ss[i])]);
    
    // non-normalized derivatives: perpendicular of vector to circle centers
    const deriv = pts.map((end,i) => end.map(pt => sub(pt,p[i])).map(w=>[-w[1],w[0]]))
    for(let side=0; side<2; side+=1) {
        let [a0,a1] = minimize_strain_hermite(pts[0][side], deriv[0][side], deriv[1][side], pts[1][side]);
        deriv[0][side] = mul(deriv[0][side], a0/3);
        deriv[1][side] = mul(deriv[1][side], a1/3);
    }
    
    const R = [pts[0][0], add(pts[0][0], deriv[0][0]), sub(pts[1][0],deriv[1][0]), pts[1][0]];
    const L = [pts[0][1], add(pts[0][1], deriv[0][1]), sub(pts[1][1],deriv[1][1]), pts[1][1]];
    
    const ans = new Path2D();
    ans.moveTo(
        ...L[0],
    );
    ans.bezierCurveTo(
        ...L[1],
        ...L[2],
        ...L[3],
    );
    ans.arc(
        ...xyr[n], 
        Math.atan2(L[3][1]-p[1][1], L[3][0]-p[1][0]), 
        Math.atan2(R[3][1]-p[1][1], R[3][0]-p[1][0]),
    );
    ans.bezierCurveTo(
        ...R[2],
        ...R[1],
        ...R[0],
    );
    ans.arc(
        ...xyr[0], 
        Math.atan2(R[0][1]-p[0][1], R[0][0]-p[0][0]),
        Math.atan2(L[0][1]-p[0][1], L[0][0]-p[0][0]),
    );
    ans.closePath();
    return ans;

}


/** Returns a vector such that pt + retval is on the Bézier curve xyr */
function nearest2(xyr, pt) {
    // returns an offset from pts[i] to the curve that would exist without it
    // assumes a relatively straight path with just one nearest point
    const eps = Number.EPSILON**0.5;
    let ans;
    for(let i=1; i<=16; i+=1) {
        const t = 0.5;
        const [f,b] = bezcut(xyr, t);
        ans = sub(b[0], pt);
        const key = dot(ans, sub(b[1],b[0]))
        if (key > eps) {
            xyr = f;
        } else if (key < -eps) {
            xyr = b;
        } else break;
    }
    return ans;
}
/** returns a vector such that pts[i] + retval is exactly on the curve without pts[i] */
function nearest(pts, i) {
    // returns an offset from pts[i] to the curve that would exist without it
    // assumes a relatively straight path with just one nearest point
    if (i-2 < 0 || i+2 >= pts.length) return pts[i].map(e=>0)
    const pt = pts[i];
    let xyr = ccr2bez(pts[i-1],pts[i-1],pts[i+1],pts[i+2]);
    fix_negative_radius(xyr);
    return nearest2(xyr, pt);
}
/** Modifies its argument to move each control point 1/2 to where the curve would be without it, but no more than a distance of cap. */
function smooth(pts, cap) {
    const delta = pts
        .slice(2,-2)
        .map((e,i) => mul(nearest(pts, i+2),0.5))
        .map(e => { const d = dot(e,e)**0.5; return d < cap ? e : mul(e, cap/d); })
        ;
    for(let i=2; i<pts.length-2; i+=1) pts[i] = add(pts[i], delta[i-2]);
}

/**
 * Given a list of points,
 * return a new list of points, generally with fewer elements,
 * that renders no more than eps different from the input.
 * 
 * When I first wrote and tested this it seemed to work, but later it didn't. No idea why
 */
function downres(pts, eps) {
    // heuristic: find points that are close to their neighbors' cuves
    // unhandled problem: fails to handle how tangents are dependent in part of neighbors
    // removing many points in a row multiplies error; only do every-other one each round, several rounds
    const rounds = 4;
    eps /= 2**rounds;
    for(let round=0; round<rounds; round+=1) {
        const mask = pts.slice(2).map((e,i) => {
            const delta = nearest(pts, i+2);
            return (dot(delta,delta) > eps*eps);
        })
        pts = pts.filter((e,i) => (i<2) || (i >= pts.length-3) || (i&1) || mask[i+2]);
        eps *= 2;
    }
    return pts
}

/**
 * With a variable-radius pen, there can be points that completely subsume other points. This function splits a stroke at any such points, removing subsumed endpoints, and returns a sequence of stroke parts with no such subsuming overlaps.
 * 
 * This version is O(n^2) but I don't anticipate strokes with so many points that that becomes an issue
 */
function splitOnBlobs(pts) {
    if (pts.length <= 1) return [pts];
    const ioverj = (i,j) => Math.hypot(pts[i][0]-pts[j][0], pts[i][1]-pts[j][1]) < pts[i][2] - pts[j][2];
    //const joveri = (i,j) => Math.hypot(pts[i][0]-pts[j][0], pts[i][1]-pts[j][1]) < pts[j][2] - pts[i][2];
    
    const gone = pts.map(e=>false);
    for(let i=0; i<pts.length; i+=1) {
        if (gone[i]) continue;
        for(let j=i-1; j>=0; j-=1) if (ioverj(i,j)) gone[j] = true;
        for(let j=i+1; j<pts.length; j+=1) if (ioverj(i,j)) gone[j] = true;
    }
    const ans = [];
    let live = [];
    for(let i=0; i<pts.length; i+=1) {
        if (gone[i]) {
            if (live.length > 0) {
                ans.push(live);
                live = [];
            }
        } else live.push(pts[i]);
    }
    if (live.length > 0) ans.push(live);
    return ans;
}


/**
 * Helper function. Given two circles, returns the endpoints of the tranezoid that bridges them.
 * @param {Array.<number>} p - [x,y,r] circle
 * @param {Array.<number>} q - [x,y,r] circle
 * @returns {Array.<?Array.<number>>} four [x,y] points; looking from p to q, they are [left side p, right side p, left side q, right side q]; or [null,null,null,null] if one circle encloses the other
 */
function trapezoid(p, q) {
    const Hx = q[0]-p[0], Hy = q[1]-p[1], Hr = q[2]-p[2];
    const ell = (Hx**2 + Hy**2);
    const Hr2 = Hr**2;
    if (ell < Hr2 || ell == 0) return Array(4).fill(null);
    const sm = Math.sqrt(ell - Hr2)/ell;
    const bm = -Hr/ell;
    const Hxb = Hx*bm, Hxs = Hx*sm, Hyb = Hy*bm, Hys = Hy*sm;
    return [
        [p[0]+p[2]*(Hxb - Hys), p[1]+p[2]*(Hyb + Hxs)],
        [p[0]+p[2]*(Hxb + Hys), p[1]+p[2]*(Hyb - Hxs)],
        [q[0]+q[2]*(Hxb - Hys), q[1]+q[2]*(Hyb + Hxs)],
        [q[0]+q[2]*(Hxb + Hys), q[1]+q[2]*(Hyb - Hxs)],
    ];
}

/**
 * A variable-width stroke class, populated by (x,y) points with radii at given time offsets.
 * WIP; ideally this will allow downsampling, converting to an envelope, etc
 */
class VWStroke {
    /**
     * @typedef {function} press2rad
     * @param {number} pressure
     * @returns {number} the pressure converted to stroke radius in world coordinates
     */
     
    /**
     * @typedef {Object} XYRT
     * @property {number} x - the x coordinate of the center of this point
     * @property {number} y - the y coordinate of the center of this point
     * @property {number} r - the radius of this point
     * @property {number} t - the time (in milliseconds) that the stroke reaches this point
     */
    
    /**
     * Create a new stroke,fixing its viewport upon creation
     * @param {DOMMatrix} matrix - The return value of ctx.getTransform() where ctx is the CanvasRenderingContext2D that would draw this stroke.
     * @param {press2rad} p2r - A function accepting pressure and returning radius; e.g p=>(p*2)**gamma/2*width
     * @param {PointerEvent} [ptrevt] - optionally, the pointerDown event that starts this stroke
     */
    constructor(matrix, p2r, ptrevt) {
        this.M = matrix.inverse();
        this.zoom = Math.hypot(matrix.a, matrix.b);
        this.p2r = p2r;
        this.t0 = ptrevt?.timeStamp || 0;
        this.pts = [];
        if (ptrevt) this.pts.push(this.xyrt(ptrevt));
    }
    
    /**
     * Helper method, intended for internal use.
     * @param {PointerEvent} ptrevt
     * @returns {XYRT}
     */
    xyrt(ptrevt) {
        return {
            x: this.M.a*ptrevt.clientX + this.M.c*ptrevt.clientY + this.M.e,
            y: this.M.b*ptrevt.clientX + this.M.d*ptrevt.clientY + this.M.f,
            r: this.p2r(ptrevt.pressure) * this.zoom,
            t: ptrevt.timeStamp - this.t0,
        };
    }
    
    /**
     * Add a point to the stroke; preferably to be called on results of getCoalescedEvents
     * @param {PointerEvent} ptrevt - a pointermove event
     * @returns {null} In the future, may return a Path2D of the most recently completed stroke segment
     */
    add(ptrevt) {
        const pt = this.xyrt(ptrevt);
        if (ptrevt.pressure == 0 && this.pts.length > 0) // workaround WebKit bug that inserts spurious 0-pressure events when pressure doesn't change
            pt.r = this.pts[this.pts.length-1].r;
        this.pts.push(pt);
        // future: return Path2D of last newly-visible bit
    }

    /// this.pts[i] with extension 1 beyond provided points
    #p(i) {
        if (i == -1) {
            if (this.pts.length < 3) return this.pts[0];
            return {
                x: 3*this.pts[0].x-3*this.pts[1].x+this.pts[2].x,
                y: 3*this.pts[0].y-3*this.pts[1].y+this.pts[2].y,
                r: 3*this.pts[0].r-3*this.pts[1].r+this.pts[2].r,
                t: 3*this.pts[0].t-3*this.pts[1].t+this.pts[2].t,
            }
        } else if (i == this.pts.length) {
            const n = this.pts.length-1;
            if (this.pts.length < 3) return this.pts[n];
            return {
                x: 3*this.pts[n].x-3*this.pts[n-1].x+this.pts[n-2].x,
                y: 3*this.pts[n].y-3*this.pts[n-1].y+this.pts[n-2].y,
                r: 3*this.pts[n].r-3*this.pts[n-1].r+this.pts[n-2].r,
                t: 3*this.pts[n].t-3*this.pts[n-1].t+this.pts[n-2].t,
            }
        } else return this.pts[i];
    }
    
    /**
     * Helper method, only used by getBezier and tailored to its needs
     * @param {...number} args - either a single index, or (index1,weight1, index2,weight2, ...)
     * @returns {Array.<number>} an [x,y,r] point, the weighted sum of the points at the given indices
     */
    #getXYR(...args) { // i, or i1 w1 i2 w2 i3 w3 ...
        if (args.length == 1) {
            const p = this.#p(args[0]);
            return [p.x, p.y, p.r];
        }
        const ans = Array(3).fill(0);
        for(let i=0; i<args.length; i+=2) {
            const p=this.#p(args[i]), w = args[i+1];
            ans[0] += p.x*w;
            ans[1] += p.y*w;
            ans[2] += p.r*w;
        }
        return ans;
    }
    
    /**
     * Helper method, intended for internal use
     * @param {number} i - the index of the first endpoint of the segment being queried
     * @returns {Array.<Array.<number>>} the four [x,y,r] control points of a cubic Bézier curve
     */
    getBezier(i) {
        if (this.pts.length == 1) return Array(4).fill(this.#getXYR(0));
        if (this.pts.length == 2) return [
            this.#getXYR(0),
            this.#getXYR(0, 2/3, 1, 1/3),
            this.#getXYR(0, 1/3, 1, 2/3),
            this.#getXYR(1),
        ];
        
        let d1 = this.#p(i).t - this.#p(i-1).t;
        let d2 = this.#p(i+1).t - this.#p(i).t;
        let d3 = this.#p(i+2).t - this.#p(i+1).t;
        // unclear: should these be raised to 0.5 power (centripetal) or not?
        let den1 = 3*d1*(d1+d2);
        let den3 = 3*d3*(d3+d2);
        // see http://www.cemyuksel.com/research/catmullrom_param/
        const ans = [
            this.#getXYR(i),
            den1>0 ? this.#getXYR(i,1, i-1,-(d2**2)/den1, i  ,(d2**2-d1**2)/den1, i+1,d1**2/den1) : this.#getXYR(i),
            den3>0 ? this.#getXYR(i+1,1, i+2,-(d2**2)/den3, i+1,(d2**2-d3**2)/den3, i  ,d3**2/den3) : this.#getXYR(i+1),
            this.#getXYR(i+1),
        ];
        return ans;
    }
    
    getWalls(i) {
        const xyr = this.getBezier(i);
        const n = xyr.length-1; // always 3
        let [L0, R0, L1, R1] = trapezoid(xyr[0], xyr[1]);
        let [L2, R2, L3, R3] = trapezoid(xyr[n-1], xyr[n]);
        if (!L0 || !L3) return [false, false]; // discontinuity; could render as pill or split, but nothing simple would maintain continuity
        
        if (false) {
            let [a0,a1] = minimize_strain_hermite(L0, sub(L1,L0), sub(L2,L3), L3);
            L1 = add(L0, mul(sub(L1,L0),a0/3));
            L2 = add(L3, mul(sub(L2,L3),a1/3));
            let [b0,b1] = minimize_strain_hermite(R0, sub(R1,R0), sub(R2,R3), R3);
            R1 = add(R0, mul(sub(R1,R0),b0/3));
            R2 = add(R3, mul(sub(R2,R3),b1/3));
        }

        return [[L0,L1,L2,L3], [R0,R1,R2,R3]];
    }
    
    envelope() {
        const ans = new Path2D();
        const side = [];
        for(let i=0; i<this.pts.length-1; i+=1) {
            let [L,R] = this.getWalls(i);
            if (L) {
                if (side.length == 0) ans.moveTo(...L[0]);
                ans.bezierCurveTo(...L[1], ...L[2], ...L[3]);
                side.push(R);
            } else {
                console.warn('skipping segment',i);
                if (side.length > 0) {
                    ans.lineTo(...side[side.length-1][3]); // fix me: should be arc cap
                    while(side.length > 0) {
                        const R = side.pop();
                        ans.bezierCurveTo(...R[2], ...R[1], ...R[0]);
                    }
                    ans.closePath(); // fix me: should be arc cap
                }
            }
        }
        if (side.length > 0) {
            ans.lineTo(...side[side.length-1][3]); // fix me: should be arc cap
            while(side.length > 0) {
                const R = side.pop();
                ans.bezierCurveTo(...R[2], ...R[1], ...R[0]);
            }
            ans.closePath(); // fix me: should be arc cap
        }
        return ans;
    }
}


/**
 * Represents one scrollable zoomable drawing workspace
 */
class Board {
    /// Pass in a BoardSet object, or at least something with two fields: canvas, an HTMLCanvasElement, and c, a CanvasRenderingContext2D for that canvas
    constructor(bs) {
        this.bs = bs;
        
        this.dx = 0; // in logical coordinates
        this.dy = 0; // in logical coordinates
        this.zoom = 720; // vertical viewpoint in logical units
        this.transform = new DOMMatrix();
        this.dirty = false;
        
        this.strokes = [];
    }
    
    /// Makes a copy of this board, rendering the same but accepting future updates separately
    clone() {
        const ans = new Board(this.bs);
        ans.dx = this.dx;
        ans.dy = this.dy;
        ans.zoom = this.zoom;
        ans.transform = this.transform;
        ans.strokes = Array.from(this.strokes); // shallow copy, OK because strokes are immutable once created
        return ans;
    }
    
    /// Requests the entire canvas be redrawn in the future
    queue_redraw() {
        if (!this.dirty) { this.dirty = true; requestAnimationFrame(ms=>this.redraw()); }
    }
    
    /// Forcibly redraws the entire canvas now. It can be slow and redundant to call this directly; queue_redraw is usually preferred
    redraw() {
        this.dirty = false;
        if (this.bs.live != this) return; // when redraw takes time (e.g. images), ignore if user has moved to new slide
        const c = this.bs.c;
        const w = this.bs.canvas.width;
        const h = this.bs.canvas.height;
        
        c.fillStyle = '#fff'
        c.fillRect(0, 0, w,h)
        c.fillStyle = '#000'

        c.save();
        c.translate(w/2, h/2);
        c.scale(h/this.zoom,h/this.zoom);
        c.translate(this.dx, this.dy);
        this.transform = c.getTransform();
        
        for(let stroke of this.strokes) {
            if ('img' in stroke) {
                c.drawImage(stroke.img, stroke.x, stroke.y, stroke.w, stroke.h);
                continue;
            } else if (!('points' in stroke)) continue;
            
            const p = stroke.points;
            c.fillStyle = stroke.color || '#fff'
            if (p.length == 1) {
                c.fill(tdot(p[0]));
            } else if (p.length == 2) {
                c.fill(tpill(p[0], p[1]));
            } else {
                c.fill(tbend(null, p[0], p[1], p[2]));
                let i;
                for(i=3; i<p.length; i+=1) {
                    c.fill(tbend(p[i-3], p[i-2], p[i-1], p[i]));
                }
                c.fill(tbend(p[i-3], p[i-2], p[i-1], null));
            }
            if (false && stroke.color) { // visualize points
                c.fillStyle = '#FFF'
                for(let xyr of stroke.points) {
                    c.beginPath();
                    c.arc(xyr[0],xyr[1],xyr[2]*0.2, 0, 2*Math.PI);
                    c.fill();
                }
            }
        }
        c.restore();
    }

    /// assumes 1 (and only 1) new point was added to stroke.points and draws the new piece
    draw_update(stroke) {
        const c = this.bs.c;
        const p = stroke.points
        c.save();
        c.fillStyle = stroke.color || '#FFF';
        c.setTransform(this.transform);

        c.fill(tdot(p[p.length-1]));
        if (p.length == 3) {
            c.fill(tbend(null, p[0], p[1], p[2]));
        } else if (p.length >= 4) {
            const i = p.length-1;
            c.fill(tbend(p[i-3], p[i-2], p[i-1], p[i]));
        }

        c.restore();
        
    }
    
    /// extract the movementX/movementY from a motion event and convert them to board coordinate space
    delta2world(evt) {
        return [
            evt.movementX * this.zoom / this.bs.canvas.clientHeight,
            evt.movementY * this.zoom / this.bs.canvas.clientHeight,
        ];
    }
    
    /// extract clientX/Y and pressure from a motion event and convert to board coordinate space
    client2world(evt, pensize, softness) {
        let x = evt.offsetX, y = evt.offsetY;
        let p = ((2*evt.pressure)**softness)/2; // keep 0.5 unchanged
        
        // put origin in center of screen, not top-left corner
        x -= this.bs.canvas.clientWidth/2
        y -= this.bs.canvas.clientHeight/2
        // zoom in or out
        x *= this.zoom/this.bs.canvas.clientHeight
        y *= this.zoom/this.bs.canvas.clientHeight
        // put origin in local pan
        x -= this.dx
        y -= this.dy
        
        // scale pen
        p *= pensize // * this.zoom / 720

        return [x,y,p]
    }
    
    /// called when a stroke is fully finished; reduces redundant data and stores for later redraw
    addStroke(data) {
        // draw the last bit that draw_update did not
        if (data.points.length >= 2) {
            const c = this.bs.c;
            c.save();
            c.fillStyle = data.color || '#FFF';
            c.setTransform(this.transform);

            if (data.points.length == 2) c.fill(tpill(...data.points));
            else c.fill(tbend(...data.points.slice(-3),null));
            
            c.restore();
        }
        
        // remove jitter
        for(let i=0; i<2; i+=1) smooth(data.points, this.zoom / 1440); // 1440 "seems to work"; maybe make configurable?
        
        // round to numbers that don't take too much space in JSON
        const digit_mult = Math.pow(10, Math.round(Math.log10(14400 / this.zoom)));
        data.points.forEach(pt => {
            pt[0] = Math.round(pt[0]*digit_mult)/digit_mult;
            pt[1] = Math.round(pt[1]*digit_mult)/digit_mult;
            pt[2] = Math.max(Math.round(pt[2]*digit_mult*10),1)/(digit_mult*10);
        });

        // remove overlaps, possibly splitting one stoke into several
        for(let p of splitOnBlobs(data.points)) {
            // p = downres(0, this.zoom / 2880); // worked in early testing but didn't hold up to more tests
            this.strokes.push({...data, points:p})
        }
    }
    
    load_image_entry(i) {
        const url = this.strokes[i].url;
        new Promise((resolve,reject) => {
            const img = new Image();
            img.onload = () => resolve(createImageBitmap(img));
            img.src = url;
        }).then(img => {
            this.strokes[i].img = img;
            this.queue_redraw();
        })
    }
    
    /// given the URL of an image, loads it, scales it, draws it, stores it for the future, and returns a Promise of a logable object
    full_size_image(url) {
        return new Promise((resolve,reject) => {
            const img = new Image();
            img.onload = () => resolve(createImageBitmap(img));
            img.src = url;
        }).then(img => {
            console.log(img);
            let w,h;
            if (img.width*9 > img.height*16) { // wide
                w = this.zoom*16/9;
                h = w*img.height/img.width;
            } else { // tall
                h = this.zoom;
                w = h*img.width/img.height;
            }
            const x = -this.dx-w/2, y = -this.dy-h/2;
            this.strokes.push({img:img, x:x, y:y, w:w, h:h});
            
            const c = this.bs.c;
            c.save();
            c.setTransform(this.transform);
            c.drawImage(img, x, y, w, h);
            c.restore();
            
            return {mode:'image', url:url, x:x, y:y, w:w, h:h};
        })
    }
}

/**
 * A canvas with a sequence of boards and methods to manipulate them.
 * Has neither event callbacks nor subscribe handlers; those are added in subclasses.
 */
class BoardSetCanvas {
    /// pass in an HTMLCanvasElement; only make one BoardSet per canvas
    constructor(c) {
        this.canvas = c;
        this.before = []; // boards we've moved past
        this.after = []; // boards we've gone back from
        this.live = new Board(this);
        this.resize();

        this.resizeObserver = new ResizeObserver(entries => this.resize(entries));
        this.resizeObserver.observe(this.canvas);
    }
    
    /// an resize observer callback
    resize(entries) {
        this.c = this.canvas.getContext('2d', {alpha:false});
        const ow = this.canvas.width;
        const oh = this.canvas.height;
        const nw = this.canvas.clientWidth * devicePixelRatio;
        const nh = this.canvas.clientHeight * devicePixelRatio;
        if (oh != nh || ow != nw) {
            this.canvas.width = nw;
            this.canvas.height = nh;
            this.c = this.canvas.getContext('2d', {alpha:false});
            this.live.queue_redraw();
        }
    }

    
    /// methods to be overridden if subclasses want to be notified when the board number of count changes
    update_boardnum() {}
    update_cursor() {}

    /**
     * All act_ functions do a key-based action, or replay it from a stored log.
     * If the action is the kind to put in a log, also returns a log entry.
     * 
     * act_zoom changes the size of the board viewport, zooming in or out
     */
    act_zoom(factor) {
        if (factor == 'reset') this.live.zoom = 720;
        else this.live.zoom *= factor;
        this.update_cursor();
        return {mode:'zoom',args:[factor]}
    }
    /// changes the center of the board viewport. See also act_zoom
    act_pan(dx,dy) {
        if (dx == 'reset') {
            this.live.dx = 0;
            this.live.dy = 0;
            return {mode:'pan',args:[dx]}
        } else {
            this.live.dx += dx;
            this.live.dy += dy;
            return {mode:'pan',args:[dx,dy]}
        }
    }
    /// changes the radius of the pen in board coordinates (not screen coordinates). See also act_zoom
    act_pensize(factor) {}
    /// changes the pressure response curve; smaller numbers are more sensitive. See also act_zoom
    act_softness(exponent) {}
    /// changes the color that will be applied to *future* strokes (not any currently being created). See also act_zoom
    act_color(color) {}
    /// toggles the visibility of the HTML overlay in front of the canvas. See also act_zoom
    act_toggle_hud() {}
    /// toggles whether the browser inverts the canvas colors or not. See also act_zoom
    act_invert_colors() {
        this.canvas.style.filter = (this.canvas.style.filter === '' ? 'invert(1)' : '');
    }
    /// moves through the stack of boards. See also act_zoom
    act_go(where) {
        if (where == 'back') {
            if (this.before.length == 0) return;
            this.after.push(this.live)
            this.live = this.before.pop()
        } else if (where == 'forward') {
            if (this.after.length == 0) return;
            this.before.push(this.live)
            this.live = this.after.pop()
        } else if (where == 'end') {
            if (this.after.length == 0) return;
            while(this.after.length > 0) {
                this.before.push(this.live)
                this.live = this.after.pop()
            }
        } else if (where == 'start') {
            if (this.before.length == 0) return;
            while(this.before.length > 0) {
                this.after.push(this.live)
                this.live = this.before.pop()
            }
        // Maybe: add where = board number
        } else {
            console.warn('unknown go argument', where)
            return;
        }
        this.update_boardnum();
        this.update_cursor();
        return {mode:'go',args:[where]};
    }
    /// duplicates a board in the stack. See also act_zoom
    act_clone(where) {
        if (where == 'forward') {
            this.before.push(this.live.clone());
        } else if (where == 'back') {
            this.after.push(this.live.clone());
        } else {
            console.warn('unknown clone destination', where)
            return;
        }
        this.update_boardnum();
        this.update_cursor();
        return {mode:'clone', args:[where]}
    }
    /// switches to a new blank board. See also act_zoom
    act_insert(where) {
        if (where == 'forward') {
            this.before.push(this.live);
            this.live = new Board(this);
        } else if (where == 'back') {
            this.after.push(this.live);
            this.live = new Board(this);
        } else {
            console.warn('unknown insert destination', where)
            return;
        }
        this.update_boardnum();
        this.update_cursor();
        return {mode:'insert', args:[where]}
    }
}

/**
 * A BoardSetCanvas with pointer and keyboard actions.
 */
class LiveBoardSet extends BoardSetCanvas {
    /// pass in an HTMLCanvasElement; only make one BoardSet per canvas
    constructor(c) {
        super(c);
        
        this.canvas.setAttribute('tabindex',0);
        requestAnimationFrame(ms => this.canvas.focus())
        
        this.logger = null;
        const logger = new WebSocket('ws://'+location.host+'/ws');
        logger.onopen = () => { this.logger = logger; console.info('websocket connected'); }
        logger.onclose = () => { if (this.logger) alert('connection to server lost; future drawing will not be logged'); this.logger = null; }
        logger.onmessage = msg => {
            msg.data.split(/(?<=})[^,{}]*(?={)/g).forEach(s => {
                const a = JSON.parse(s)
                if (a.mode == 'draw') { this.live.strokes.push(a)
                } else if (a.mode == 'erase') { this.live.strokes.push(a)
                } else if (a.mode == 'image') {
                    this.live.strokes.push(a)
                    this.live.load_image_entry(this.live.strokes.length-1)
                } else if ('act_'+a.mode in this) {
                    this['act_'+a.mode](...a.args)
                } else {
                    console.warn("unexpected log entry", a);
                }
            })
            this.live.queue_redraw()
        }
        
        this.pointers = {};
        this.color = defaults.get('color');
        this.pensize = defaults.get('pensize');
        this.softness = defaults.get('softness');
        
        this.canvas.addEventListener('pointermove', e=>this.move(e) )
        this.canvas.addEventListener('pointerup', e =>this.up(e) )
        this.canvas.addEventListener('pointerdown', e=>this.down(e) )
        this.canvas.addEventListener('pointerleave', e=> this.up(e) )
        this.canvas.addEventListener('pointerenter', e=>{ if (e.buttons) { this.down(e) }} )
        this.canvas.addEventListener('keydown', e=>this.keyboard(e) )
        // this.canvas.addEventListener('paste', e=>this.imagepaste(e) ) // error: requires contenteditable which messes up focus and key listeners
    }
    
    async imagepaste(e) { // unfinished
        e.preventDefault()
        for(let item of e.clipboardData.items) {
            if (item.type.startsWith('image/')) {
                const reader = new FileReader();
                const file = item.getAsFile();
                reader.addEventListener('load', e=>{console.debug(reader.result)}) // demo, unfinished
                reader.readAsDataURL(file);
            }
        }
    }

    newDefaults() {
        this.color = defaults.get('color');
        this.pensize = defaults.get('pensize');
        this.softness = defaults.get('softness');
    }
    
    colorMaybeInverted() {
        if (document.getElementById('cursor').style.filter == 'invert(1)' && this.color[0] == '#') {
            return this.color.replace(/[A-F0-9]/gi, c=>({'F':0,'E':1,'D':2,'C':3,'B':4,'A':5,'9':6,'8':7,'7':8,'6':9,'5':'A','4':'B','3':'C','2':'D','1':'E','0':'F'}[c.toUpperCase()]))
        }
        return this.color;
    }
    

    /// pointerdown (and pointerenter) event listener callback
    down(evt) {
        if (evt.pointerId in this.pointers) { console.error("Duplicate down!",evt.pointerId, evt); return; }
        if (evt.shiftKey) {
            this.pointers[evt.pointerId] = {mode:'pan',args:[0,0]};
        } else if ((evt.buttons & 0b100000) || (evt.button === 5)) {
            const xyr = this.live.client2world(evt,this.pensize*5, 1);
            this.pointers[evt.pointerId] = {mode:'erase', points:[xyr]};
            this.live.draw_update(this.pointers[evt.pointerId]);
            if (this.logger) this.logger.send(JSON.stringify({live:true,ts:evt.timeStamp,evt:'down',pid:evt.pointerId,xyr:xyr,c:'erase'}))
        } else {
            const xyr = this.live.client2world(evt,this.pensize, this.softness);
            this.pointers[evt.pointerId] = {mode:'draw', color:this.color, points:[xyr]};
            this.live.draw_update(this.pointers[evt.pointerId]);
            if (this.logger) this.logger.send(JSON.stringify({live:true,ts:evt.timeStamp,evt:'down',pid:evt.pointerId,xyr:xyr,c:this.color}))
        }
    }
    /// pointerup (and pointerleave) event listener callback
    up(evt) {
        if (evt.pointerId in this.pointers) {
            const data = this.pointers[evt.pointerId];
            delete this.pointers[evt.pointerId];
            if (data.mode === 'draw' || data.mode === 'erase') {
                const osl = this.live.strokes.length;
                this.live.addStroke(data);
                if (this.logger) {
                    for(let stroke of this.live.strokes.slice(osl))
                        this.logger.send(JSON.stringify(stroke));
                }
                if (this.logger) this.logger.send(JSON.stringify({live:true,ts:evt.timeStamp,evt:'up',pid:evt.pointerId}))
            } else if (data.mode === 'pan') {
                if (this.logger) this.logger.send(JSON.stringify(data));
            }
        } else { /* duplicate up occurs on leave events; ignore */ }
    }
    /// pointermove event listener callback
    move(evts) {
        if (evts.pointerId in this.pointers) {
            const data = this.pointers[evts.pointerId];
            if (data.mode === 'draw' || data.mode === 'erase') {
                const pmult = (data.mode === 'erase' ? 5 : 1)
                const ppow = (data.mode === 'erase' ? 1 : this.softness)
                let cnt = 0;
                for(let evt of evts.getCoalescedEvents()) {
                    if (evt.pressure === 0) continue
                    /*
                    // on fireFox, evt's movementX/movementY are not deltas, but evts's are
                    const dt = evt.timeStamp - window.lastTS
                    const speed = Math.hypot(evt.movementX, evt.movementY) / dt;
                    console.log(dt.toFixed(1), speed, evts.movementX, evts.movementY)
                    window.lastTS = evt.timeStamp
                    */
                    cnt += 1;
                    const xyr = this.live.client2world(evt,this.pensize * pmult, ppow);
                    data.points.push(xyr);
                    this.live.draw_update(this.pointers[evt.pointerId]);
                }
                if (this.logger) this.logger.send(JSON.stringify({live:true,ts:evts.timeStamp,evt:'move',pid:evts.pointerId,xyr:data.points.slice(-cnt)}))
            } else if (data.mode === 'pan') {
                const [dx,dy] = this.live.delta2world(evts);
                data.args[0] += dx;
                data.args[1] += dy;
                this.live.dx += dx;
                this.live.dy += dy;
                this.live.queue_redraw();
                if (this.logger) this.logger.send(JSON.stringify({live:true,ts:evts.timeStamp,evt:'pan',delta:[dx,dy]}))
            }
        } else { /* do nothing: hovering */ }
    }
    
    /// keydown event listener callback
    keyboard(e) {
        const lookup = (e.key.length > 1 && e.shiftKey?'Shift+':'')+(e.ctrlKey?'Ctrl+':'')+(e.altKey?'Alt+':'')+(e.metaKey?'Meta+':'')+e.key;
        if (window.shortcuts.has(lookup)) {
            for(let action of window.shortcuts.get(lookup)) {
                if ('act_'+action[0] in this) {
                    let loggable = this['act_'+action[0]](...action.slice(1));
                    if (loggable) this.live.queue_redraw()
                    if (loggable && this.logger) this.logger.send(JSON.stringify(loggable))
                    if (this.logger) this.logger.send(JSON.stringify({live:true,ts:e.timeStamp,evt:'act',action:action}))
                }
                else console.warn('unknown key binding',action)
            }
        }// else console.debug(lookup)
    }
    
    /// a helper method to update the board number display
    update_boardnum() {
        document.getElementById('page').textContent = `${this.before.length+1} of ${this.before.length+1+this.after.length}`;
    }
    update_cursor() {
        document.getElementById('cursor').style.height = (100 * this.pensize / this.live.zoom) + 'vh'
    }

    /// inserts an image as the initial contents of a new slide
    act_putimage() {
        const url = prompt("Image URL")
        if (!url) return;
        this.live.full_size_image(url).then(obj => {
            if(this.logger) this.logger.send(JSON.stringify(obj));
        });
    }

    /// changes the radius of the pen in board coordinates (not screen coordinates). See also act_zoom
    act_pensize(factor) {
        if (factor == 'reset') this.pensize = defaults.get('pensize');
        else this.pensize *= factor;
        this.update_cursor();
    }
    /// changes the pressure response curve; smaller numbers are more sensitive. See also act_zoom
    act_softness(exponent) {
        this.softness = exponent;
    }
    /// changes the color that will be applied to *future* strokes (not any currently being created). See also act_zoom
    act_color(color) {
        this.color = color;
        document.getElementById('cursor').setAttribute('fill', color)
    }
    /// toggles the visibility of the HTML overlay in front of the canvas. See also act_zoom
    act_toggle_hud() {
        const hud = document.getElementById('hud');
        hud.style.display = hud.style.display == 'none' ? '' : 'none';
    }
    /// toggles whether the browser inverts the canvas colors or not. See also act_zoom
    act_invert_colors() {
        super.act_invert_colors();
        const cusor = document.getElementById('cursor');
        cusor.style.filter = (cusor.style.filter === '' ? 'invert(1)' : '');
    }
}

/**
 * A BoardSetCanvas accepting actions from a websocket actions.
 */
class LurkerBoardSet extends BoardSetCanvas {
    /// pass in an HTMLCanvasElement; only make one BoardSet per canvas
    constructor(c) {
        super(c);
        
        this.lurker = null;
        const lurker = new WebSocket('ws://'+location.host+'/lurkws');
        lurker.onopen = () => { this.lurker = lurker; console.info('websocket connected'); }
        lurker.onclose = () => { if (this.lurker) alert('connection to server lost; no future updates will be shown'); this.lurker = null; }
        lurker.onmessage = msg => {
            let redraw = false;
            msg.data.split(/(?<=})[^,{}]*(?={)/g).forEach(s => {
                const a = JSON.parse(s)
                if (a.live) {
                    if (a.evt == 'act') {
                        if ('act_'+a.action[0] in this) {
                            let loggable = this['act_'+a.action[0]](...a.action.slice(1));
                            if (loggable) redraw = true;
                        }
                        else console.warn('unknown action',a.action)
                    } else if (a.evt == 'down') {
                        if (a.c == 'erase') this.pointers[a.pid] = {mode:'erase', points:[a.xyr]};
                        else this.pointers[a.pid] = {mode:'draw', color:a.c, points:[a.xyr]};
                        this.live.draw_update(this.pointers[a.pid]);
                    } else if (a.evt == 'up') {
                        this.live.addStroke(this.pointers[a.pid]);
                        delete this.pointers[a.pid];
                    } else if (a.evt == 'move') {
                        for(let xyr of a.xyr) {
                            this.pointers[a.pid].points.push(xyr);
                            this.live.draw_update(this.pointers[a.pid]);
                        }
                    } else if (a.evt == 'pan') {
                        this.live.dx += a.delta[0];
                        this.live.dy += a.delta[1];
                        redraw = true;
                    } else {
                        console.warn("unexpected live entry", a);
                    }
                } else if (a.mode == 'draw') { this.live.strokes.push(a); redraw=true;
                } else if (a.mode == 'erase') { this.live.strokes.push(a); redraw=true;
                } else if ('act_'+a.mode in this) {
                    this['act_'+a.mode](...a.args)
                } else {
                    console.warn("unexpected log entry", a);
                }
            })
            if (redraw) this.live.queue_redraw()
        }
        
        this.pointers = {};
    }
}



/// Given a kayboard short string, returns a Map of parses shortcuts
function parse_shortcuts(s) {
    let ans = new Map();
    const def = new Map([['pensize',4], ['softness',0.5], ['color','#000']]);
    for(let line of s.split(/[\r\n]+/g)) {
        if (line.trim() == '') continue;
        let [key,command] = (line[0] == ' ') ? [' ', line.trim()] : line.trim().split(/\s+(.*)/,2);
        if (key != '+' && key.includes('+')) {
            let bits = key.split('+');
            if (bits[bits.length-1] == '' && bits[bits.length-2] == '') bits.splice(-2,2,'+');
            key = bits.pop()
            let idx;
            for(let mod of ['Meta', 'Alt', 'Ctrl', 'Shift']) {
                if ((idx = bits.indexOf(mod)) >= 0) key = bits.splice(idx,1)[0] + '+' + key;
            }
            if (bits.length > 0) console.warn('Ignoring unknown modifiers', bits);
        }
        command = command.split(/\s+/).map(e => { const n = Number(e); return Number.isNaN(n) ? e : n; })
        if (key == 'default') {
            console.log(key, command);
            if (command.length == 2 && def.has(command[0])) {
                def.set(command[0], command[1]);
            } else {
                console.warn('Ignoring unsupported default:', line);
            }
            console.log(def);
            continue;
        }
        if (!ans.has(key)) ans.set(key, []);
        ans.get(key).push(command);
    }
    return [ans,def];
}

var [shortcuts,defaults] = parse_shortcuts(def_keys);
fetch('keys.txt', {'cache':'no-store'}).then(res => res.text()).then(parse_shortcuts).then(([m,d]) => {
    window.shortcuts = m;
    window.defaults = d;
    document.querySelectorAll('canvas').forEach(c=>c.tool?.newDefaults())
});



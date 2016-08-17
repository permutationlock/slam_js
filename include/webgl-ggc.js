// webgl-ggc.js
// VERSION 0.6
// Glenn G. Chappell
// 4 Dec 2013
//
// For CS 381 Fall 2013
// GGC's WebGL Utilities
// Uses J3DIMath.js for matrix type (J3DIMatrix4)
//
// History:
// - v0.1-0.4: Miscellaneous foolishness.
// - v0.5: Write drawSquare, drawCube, drawSphere, drawCylinder.
//         Add optional alpha param to drawXxx functions.
// - v0.6: Fix incorrect texture coordinates in shapes.

// Internal-use items have names ending with '_internal_'. All others
// are available for external use.

// Assumptions about WebGL contexts:
// - A context passed to any of the functions in this file will have
//   members pMatrix, mvMatrix, and tMatrix, which are J3DIMatrix4
//   objects. (The idea is that these represent the projection matrix,
//   model/view matrix, and texture matrix, respectively.)
// Note that function getGlContext creates these members in the returned
// context, and sets them to identity matrices.

// Assumptions about shaders:
// - Vertex coordinates, color, normal vector, texture coordinates, and
//   tangent vector, IF THESE ARE USED, are the respective variables as
//   follows:
//
//     attribute vec4 vertex_attr;
//     attribute vec4 color_attr;
//     attribute vec3 normal_attr;
//     attribute vec4 texcoord_attr;
//     attribute vec3 tangent_attr;
//
// - The projection matrix, model/view matrix, normal matrix, and
//   texture matrix, IF THESE ARE USED, are the respective variables as
//   follows:
//
//     uniform mat4 projectionMatrix;
//     uniform mat4 modelViewMatrix;
//     uniform mat3 normalMatrix;
//     uniform mat4 textureMatrix;
//
// Note that, if a variable is not used in a shader, then it need not be
// declared.


// errOut
// Given string, attempts to print it as an error message.
// - First tries to find HTML element with Id = 'err'. If found, appends
//   message to its content.
// - Otherwise, tries to log message to window console.
// - If all the above fail, then does nothing.
function errOut(msg)
{
    var f = arguments.callee;  // Current function
    if (!f.inited)
    {
        f.inited = true;
        f.which = 0;
    }
    ++f.which;
    var fullmsg = '[APPLICATION ERROR] ' + msg;

    var e = document.getElementById('err');
    if (e)
    {
        e.innerHTML +=
            encodeStringForHTMLText_internal_(fullmsg) + '<br>';
        return;
    }

    if ('console' in window &&
        'error' in window.console &&
        typeof window.console.error == 'function')
    {
        window.console.error(fullmsg);
        return;
    }
}


// encodeStringForHTMLText_internal_
// Given an ASCII string, encode it so that it can be included as
// viewable text in an HTML document. Returns encoded string.
function encodeStringForHTMLText_internal_(str)
{
    return str
        .replace(/\&/g, '&amp;')
        .replace(/\</g, '&lt;')
        .replace(/\>/g, '&gt;');
}


// getElapsedTime
// Returns number of seconds (to nearest thousandth?) since previous
// call. Returns 0.0 on first call. If max is given, limits returned
// values to at most max.
function getElapsedTime(max)
{
    var f = arguments.callee;  // Current function
    if (!f.inited)
    {
        f.inited = true;
        f.savetime = new Date();
        return 0.;
    }
    else
    {
        var oldtime = f.savetime;
        f.savetime = new Date();
        var etime = (f.savetime - oldtime) / 1000.;
        if (typeof max != 'undefined' && etime > max)
            return max;
        return etime;
    }
}


// getMousePos
// Given a canvas and an event object, return JS object holding mouse
// pos in canvas coordinates (x & y members of object).
function getMousePos(canvas, evt)
{
    if (!canvas || !canvas.getBoundingClientRect)
    {
        errOut(arguments.callee.name + ': ' +
               'No canvas.getBoundingClientRect');
        return null;
    }
    if (!evt || !evt.clientX || !evt.clientY)
    {
        errOut(arguments.callee.name + ': ' +
               'No event passed');
        return null;
    }

    var rect = canvas.getBoundingClientRect();
    return {
        x: evt.clientX-rect.left,
        y: evt.clientY-rect.top
    };
}


// whereAmI
// Given matrix cammat, finds the point that cammat takes to (the
// homogeneous form of) the origin. Thus, if cammat is the camera-
// transformation matrix, then we return the camera position in world
// coordinates.
//
// Return value is JS object with camera-position coordinates in x, y, z
// members.
function whereAmI(cammat)
{
    var mat = new J3DIMatrix4(cammat);
    mat.invert();
    var arr = mat.getAsFloat32Array();
    return {
        x: arr[12]/arr[15],
        y: arr[13]/arr[15],
        z: arr[14]/arr[15]
    };
}


// checkContext_internal_
// Checks whether ctx is a valid WebGL context. If not, signals error
// using given function name.
function checkContext_internal_(ctx, funcname)
{
    if (typeof ctx != 'object' || !('clearColor' in ctx))
    {
        errOut(funcname + ': ' +
               'Called without valid WebGL context');
        return false;
    }
    return true;
}


// getProgram_internal_
// Given WebGL context, returns active shader program object, or null if
// none. If none, signals error using given function name.
function getProgram_internal_(ctx, funcname)
{
    if (!checkContext_internal_(ctx, funcname))
        return null;
    var shaderProgram = ctx.getParameter(ctx.CURRENT_PROGRAM);
    if (!shaderProgram)
    {
        errOut(funcname + ': ' +
               'Called with context having no active shaders');
        return null;
    }
    return shaderProgram;
}


// getAttribLocs
// Given WebGL context, returns locations of standard attribute
// variables in an array. Each array item is attribute location or -1 if
// attribute not found in shaders. Attributes + their indices are as
// follows:
//
//     Index   Attribute
//       0     vertex_attr
//       1     color_attr
//       2     normal_attr
//       3     texcooord_attr
//       4     tangent_attr
//
// If no active shaders, signals error, returns null.
function getAttribLocs(ctx)
{
    var shaderProgram =
        getProgram_internal_(ctx, arguments.callee.name);
    if (!shaderProgram)
        return null;

    return [
        ctx.getAttribLocation(shaderProgram, 'vertex_attr'),
        ctx.getAttribLocation(shaderProgram, 'color_attr'),
        ctx.getAttribLocation(shaderProgram, 'normal_attr'),
        ctx.getAttribLocation(shaderProgram, 'texcoord_attr'),
        ctx.getAttribLocation(shaderProgram, 'tangent_attr')
    ];
}


// drawSquare
// Draw solid square with given side length, centered at origin, in
// x,y-plane, axis-aligned.
//
// ctx is WebGL context. r, g, b, a are optional arguments giving color.
// Uses standard attribute, uniform shader variable names; see comments
// @ beginning of this file.
function drawSquare(ctx,
                    size,
                    r, g, b, a)
{
    // Get attribute locations
    var attriblocs = getAttribLocs(ctx);
    if (!attriblocs)
    {
        errOut(arguments.callee.name + ': ' +
               'Could not get attribute locations');
        return;
    }

    // Set up parameters
    if (typeof r != 'number') r = 0.7;
    if (typeof g != 'number') g = 0.7;
    if (typeof b != 'number') b = 0.7;
    if (typeof a != 'number') a = 1.0;

    // Create VBOs
    var buffs = new Array(5);
    var datas = new Array(5);
    for (var i = 0; i < 5; ++i)
    {
        buffs[i] = ctx.createBuffer();
        var components = (i == 2 || i == 4) ? 3 : 4;
        datas[i] = new Float32Array(components*4);
    }
    for (var i = 0; i < 4; ++i)
    {
        var x = (i == 1 || i == 2) ? 1. : 0.;
        var y = (i == 2 || i == 3) ? 1. : 0.;

        var b4 = 4*i;  // Base for indices
        var b3 = 3*i;

        // vertex coords
        datas[0][b4+0] = (x-0.5) * size;
        datas[0][b4+1] = (y-0.5) * size;
        datas[0][b4+2] = 0.;
        datas[0][b4+3] = 1.;

        // color
        datas[1][b4+0] = r;
        datas[1][b4+1] = g;
        datas[1][b4+2] = b;
        datas[1][b4+3] = a;

        // normal
        datas[2][b3+0] = 0.;
        datas[2][b3+1] = 0.;
        datas[2][b3+2] = 1.;

        // texture coords
        datas[3][b4+0] = x;
        datas[3][b4+1] = y;
        datas[3][b4+2] = 0.;
        datas[3][b4+3] = 1.;

        // tangent
        datas[4][b3+0] = 1.;
        datas[4][b3+1] = 0.;
        datas[4][b3+2] = 0.;
    }
    for (var i in attriblocs)
    {
        if (attriblocs[i] == -1)
            continue;
        var components = (i == 2 || i == 4) ? 3 : 4;
        ctx.bindBuffer(ctx.ARRAY_BUFFER, buffs[i]);
        ctx.bufferData(
            ctx.ARRAY_BUFFER, datas[i], ctx.STATIC_DRAW);
        ctx.vertexAttribPointer(
            attriblocs[i], components, ctx.FLOAT, false, 0, 0);
    }

    // Set up uniforms, enable attributes
    sendMatrices(ctx);
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.enableVertexAttribArray(attriblocs[i]);

    // Draw with VBO
    ctx.drawArrays(ctx.TRIANGLE_FAN, 0, 4);

    // Disable attributes
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.disableVertexAttribArray(attriblocs[i]);

    // Delete buffer objects
    for (i in buffs)
        ctx.deleteBuffer(buffs[i]);
}


// drawCube
// Draw solid cube -- like glutSolidCube.
//
// ctx is WebGL context. r, g, b, a are optional arguments giving color.
// Uses standard attribute, uniform shader variable names; see comments
// @ beginning of this file.
function drawCube(ctx,
                  size,
                  r, g, b, a)
{
    // Set up parameters
    if (typeof r != 'number') r = 0.7;
    if (typeof g != 'number') g = 0.7;
    if (typeof b != 'number') b = 0.7;
    if (typeof a != 'number') a = 1.0;

    // +x face
    pushMvMatrix(ctx);
    ctx.mvMatrix.rotate(90., 0.,1.,0.);
    ctx.mvMatrix.translate(0., 0., size/2.);
    drawSquare(ctx, size, r, g, b, a);
    popMvMatrix(ctx);

    // -x face
    pushMvMatrix(ctx);
    ctx.mvMatrix.rotate(-90., 0.,1.,0.);
    ctx.mvMatrix.translate(0., 0., size/2.);
    drawSquare(ctx, size, r, g, b, a);
    popMvMatrix(ctx);

    // +y face
    pushMvMatrix(ctx);
    ctx.mvMatrix.rotate(-90., 1.,0.,0.);
    ctx.mvMatrix.translate(0., 0., size/2.);
    drawSquare(ctx, size, r, g, b, a);
    popMvMatrix(ctx);

    // -y face
    pushMvMatrix(ctx);
    ctx.mvMatrix.rotate(90., 1.,0.,0.);
    ctx.mvMatrix.translate(0., 0., size/2.);
    drawSquare(ctx, size, r, g, b, a);
    popMvMatrix(ctx);

    // +z face
    pushMvMatrix(ctx);
    ctx.mvMatrix.translate(0., 0., size/2.);
    drawSquare(ctx, size, r, g, b, a);
    popMvMatrix(ctx);

    // -z face
    pushMvMatrix(ctx);
    ctx.mvMatrix.rotate(180., 0.,1.,0.);
    ctx.mvMatrix.translate(0., 0., size/2.);
    drawSquare(ctx, size, r, g, b, a);
    popMvMatrix(ctx);
}


// drawSphere
// Draw solid sphere -- like glutSolidSphere.
//
// ctx is WebGL context. r, g, b, a are optional arguments giving color.
// Uses standard attribute, uniform shader variable names; see comments
// @ beginning of this file.
function drawSphere(ctx,
                    radius, slices, stacks,
                    r, g, b, a)
{
    // Get attribute locations
    var attriblocs = getAttribLocs(ctx);
    if (!attriblocs)
    {
        errOut(arguments.callee.name + ': ' +
               'Could not get attribute locations');
        return;
    }

    // Set up parameters
    if (typeof r != 'number') r = 0.7;
    if (typeof g != 'number') g = 0.7;
    if (typeof b != 'number') b = 0.7;
    if (typeof a != 'number') a = 1.0;

    // Create VBOs
    var buffs = new Array(5);
    var datas = new Array(5);
    for (var i = 0; i < 5; ++i)
    {
        buffs[i] = ctx.createBuffer();
        var components = (i == 2 || i == 4) ? 3 : 4;
        datas[i] = new Float32Array(components*(slices+1)*(stacks+1));
    }
    for (var i1 = 0; i1 <= stacks; ++i1)
    {
        var tc1 = i1/stacks;
        var ang1 = tc1 * Math.PI;
        for (var i2 = 0; i2 <= slices; ++i2)
        {
            var tc2 = i2/slices;
            var ang2 = tc2 * 2.*Math.PI;

            var nx = Math.cos(ang2)*Math.sin(ang1);
            var ny = Math.sin(ang2)*Math.sin(ang1);
            var nz = Math.cos(ang1);

            var b4 = 4*(i1*(slices+1) + i2);  // Base for indices
            var b3 = 3*(i1*(slices+1) + i2);

            // vertex coords
            datas[0][b4+0] = radius * nx;
            datas[0][b4+1] = radius * ny;
            datas[0][b4+2] = radius * nz;
            datas[0][b4+3] = 1.;

            // color
            datas[1][b4+0] = r;
            datas[1][b4+1] = g;
            datas[1][b4+2] = b;
            datas[1][b4+3] = a;

            // normal
            datas[2][b3+0] = nx
            datas[2][b3+1] = ny
            datas[2][b3+2] = nz

            // texture coords
            datas[3][b4+0] = tc1;
            datas[3][b4+1] = tc2;
            datas[3][b4+2] = 0.;
            datas[3][b4+3] = 1.;

            // tangent
            datas[4][b3+0] = Math.cos(ang2)*Math.cos(ang1);
            datas[4][b3+1] = Math.sin(ang2)*Math.cos(ang1);
            datas[4][b3+2] = -Math.sin(ang1);
        }
    }
    for (var i in attriblocs)
    {
        if (attriblocs[i] == -1)
            continue;
        var components = (i == 2 || i == 4) ? 3 : 4;
        ctx.bindBuffer(ctx.ARRAY_BUFFER, buffs[i]);
        ctx.bufferData(
            ctx.ARRAY_BUFFER, datas[i], ctx.STATIC_DRAW);
        ctx.vertexAttribPointer(
            attriblocs[i], components, ctx.FLOAT, false, 0, 0);
    }

    // Set up uniforms, enable attributes
    sendMatrices(ctx);
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.enableVertexAttribArray(attriblocs[i]);

    // Draw with EBO
    var ebuff = ctx.createBuffer();
    var edata = new Uint16Array(2*(slices+1));
    for (var i1 = 0; i1 < stacks; ++i1)
    {
        for (var i2 = 0; i2 <= slices; ++i2)
        {
            edata[2*i2+0] = i1*(slices+1) + i2;
            edata[2*i2+1] = (i1+1)*(slices+1) + i2;
        }

        ctx.bindBuffer(ctx.ELEMENT_ARRAY_BUFFER, ebuff);
        ctx.bufferData(
            ctx.ELEMENT_ARRAY_BUFFER, edata, ctx.DYNAMIC_DRAW);
        ctx.drawElements(
            ctx.TRIANGLE_STRIP, 2*(slices+1), ctx.UNSIGNED_SHORT, 0);
    }

    // Disable attributes
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.disableVertexAttribArray(attriblocs[i]);

    // Delete buffer objects
    for (i in buffs)
        ctx.deleteBuffer(buffs[i]);
    ctx.deleteBuffer(ebuff);
}


// drawTorus
// Draw solid torus -- like glutSolidTorus.
//
// ctx is WebGL context. r, g, b, a are optional arguments giving color.
// Uses standard attribute, uniform shader variable names; see comments
// @ beginning of this file.
function drawTorus(ctx,
                   smrad, bigrad, smsubdivs, bigsubdivs,
                   r, g, b, a)
{
    // Get attribute locations
    var attriblocs = getAttribLocs(ctx);
    if (!attriblocs)
    {
        errOut(arguments.callee.name + ': ' +
               'Could not get attribute locations');
        return;
    }

    // Set up parameters
    if (typeof r != 'number') r = 0.7;
    if (typeof g != 'number') g = 0.7;
    if (typeof b != 'number') b = 0.7;
    if (typeof a != 'number') a = 1.0;

    // Create VBOs
    var buffs = new Array(5);
    var datas = new Array(5);
    for (var i = 0; i < 5; ++i)
    {
        buffs[i] = ctx.createBuffer();
        var components = (i == 2 || i == 4) ? 3 : 4;
        datas[i] = new Float32Array(components*(smsubdivs+1)*(bigsubdivs+1));
    }
    for (var i1 = 0; i1 <= bigsubdivs; ++i1)
    {
        var tc1 = i1/bigsubdivs;
        var ang1 = tc1 * 2.0*Math.PI;
        for (var i2 = 0; i2 <= smsubdivs; ++i2)
        {
            var tc2 = i2/smsubdivs;
            var ang2 = tc2 * 2.0*Math.PI;

            var b4 = 4*(i1*(smsubdivs+1) + i2);  // Base for indices
            var b3 = 3*(i1*(smsubdivs+1) + i2);

            // vertex coords
            datas[0][b4+0] =
                (bigrad+smrad*Math.cos(ang2))*Math.cos(ang1);
            datas[0][b4+1] =
                (bigrad+smrad*Math.cos(ang2))*Math.sin(ang1);
            datas[0][b4+2] =
                smrad*Math.sin(ang2);
            datas[0][b4+3] =
                1.;

            // color
            datas[1][b4+0] = r;
            datas[1][b4+1] = g;
            datas[1][b4+2] = b;
            datas[1][b4+3] = a;

            datas[2][b3+0] = Math.cos(ang2)*Math.cos(ang1);
            datas[2][b3+1] = Math.cos(ang2)*Math.sin(ang1);
            datas[2][b3+2] = Math.sin(ang2);

            // texture coords
            datas[3][b4+0] = tc1;
            datas[3][b4+1] = tc2;
            datas[3][b4+2] = 0.;
            datas[3][b4+3] = 1.;

            // tangent
            datas[4][b3+0] = -Math.sin(ang1);
            datas[4][b3+1] = Math.cos(ang1);
            datas[4][b3+2] = 0.;
        }
    }
    for (var i in attriblocs)
    {
        if (attriblocs[i] == -1)
            continue;
        var components = (i == 2 || i == 4) ? 3 : 4;
        ctx.bindBuffer(ctx.ARRAY_BUFFER, buffs[i]);
        ctx.bufferData(
            ctx.ARRAY_BUFFER, datas[i], ctx.STATIC_DRAW);
        ctx.vertexAttribPointer(
            attriblocs[i], components, ctx.FLOAT, false, 0, 0);
    }

    // Set up uniforms, enable attributes
    sendMatrices(ctx);
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.enableVertexAttribArray(attriblocs[i]);

    // Draw with EBO
    var ebuff = ctx.createBuffer();
    var edata = new Uint16Array(2*(smsubdivs+1));
    for (var i1 = 0; i1 < bigsubdivs; ++i1)
    {
        for (var i2 = 0; i2 <= smsubdivs; ++i2)
        {
            edata[2*i2+0] =
                i1*(smsubdivs+1) + i2;
            edata[2*i2+1] =
                (i1+1)*(smsubdivs+1) + i2;
        }

        ctx.bindBuffer(ctx.ELEMENT_ARRAY_BUFFER, ebuff);
        ctx.bufferData(
            ctx.ELEMENT_ARRAY_BUFFER, edata, ctx.DYNAMIC_DRAW);
        ctx.drawElements(
            ctx.TRIANGLE_STRIP, 2*(smsubdivs+1), ctx.UNSIGNED_SHORT, 0);
    }

    // Disable attributes
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.disableVertexAttribArray(attriblocs[i]);

    // Delete buffer objects
    for (i in buffs)
        ctx.deleteBuffer(buffs[i]);
    ctx.deleteBuffer(ebuff);
}


// drawCylinder
// Draw a cylinder, radius 1, length 2, centered at origin, around
// x-axis. Uses given number of subdivisions.
//
// ctx is WebGL context. r, g, b, a are optional arguments giving color.
// Uses standard attribute, uniform shader variable names; see comments
// @ beginning of this file.
function drawCylinder(ctx,
                      subdivs,
                      r, g, b, a)
{
    // Get attribute locations
    var attriblocs = getAttribLocs(ctx);
    if (!attriblocs)
    {
        errOut(arguments.callee.name + ': ' +
               'Could not get attribute locations');
        return;
    }

    // Set up parameters
    if (typeof r != 'number') r = 0.7;
    if (typeof g != 'number') g = 0.7;
    if (typeof b != 'number') b = 0.7;
    if (typeof a != 'number') a = 1.0;
    var halflen = 1.;     // Half of cylinder's length
    var radius = 1.;      // Cylinder's radius
    var sd1 = subdivs;    // Subdivisions along cylinder
    var sd2 = 4*subdivs;  // Subdivisions around cylinder

    // Create VBOs

    var buffs = new Array(5);
    var datas = new Array(5);
    for (var i = 0; i < 5; ++i)
    {
        buffs[i] = ctx.createBuffer();
        var components = (i == 2 || i == 4) ? 3 : 4;
        datas[i] = new Float32Array(components*(sd2+1)*(sd1+1));
    }

    for (var i1 = 0; i1 <= sd1; ++i1)
    {
        var tc1 = i1/sd1;
        var x = halflen - tc1 * 2.*halflen;
        for (var i2 = 0; i2 <= sd2; ++i2)
        {
            var tc2 = i2/sd2;
            var ang2 = tc2 * 2.*Math.PI;
            var ny = Math.cos(ang2);
            var nz = Math.sin(ang2);

            var b4 = 4*(i1*(sd2+1) + i2);  // Base for indices
            var b3 = 3*(i1*(sd2+1) + i2);

            // vertex coords
            datas[0][b4+0] = x;
            datas[0][b4+1] = radius * ny;
            datas[0][b4+2] = radius * nz;
            datas[0][b4+3] = 1.;

            // color
            datas[1][b4+0] = r;
            datas[1][b4+1] = g;
            datas[1][b4+2] = b;
            datas[1][b4+3] = a;

            // normal
            datas[2][b3+0] = 0.;
            datas[2][b3+1] = ny;
            datas[2][b3+2] = nz;

            // texture coords
            datas[3][b4+0] = tc1;
            datas[3][b4+1] = tc2;
            datas[3][b4+2] = 0.;
            datas[3][b4+3] = 1.;

            // tangent
            datas[4][b3+0] = 1.;
            datas[4][b3+1] = 0.;
            datas[4][b3+2] = 0.;
        }
    }
    for (var i in attriblocs)
    {
        if (attriblocs[i] == -1)
            continue;
        var components = (i == 2 || i == 4) ? 3 : 4;
        ctx.bindBuffer(ctx.ARRAY_BUFFER, buffs[i]);
        ctx.bufferData(
            ctx.ARRAY_BUFFER, datas[i], ctx.STATIC_DRAW);
        ctx.vertexAttribPointer(
            attriblocs[i], components, ctx.FLOAT, false, 0, 0);
    }

    // Set up uniforms, enable attributes
    sendMatrices(ctx);
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.enableVertexAttribArray(attriblocs[i]);

    // Draw with EBO
    var ebuff = ctx.createBuffer();
    var edata = new Uint16Array(2*(sd2+1));
    for (var i1 = 0; i1 < sd1; ++i1)
    {
        for (var i2 = 0; i2 <= sd2; ++i2)
        {
            edata[2*i2+0] = i1*(sd2+1) + i2;
            edata[2*i2+1] = (i1+1)*(sd2+1) + i2;
        }

        ctx.bindBuffer(ctx.ELEMENT_ARRAY_BUFFER, ebuff);
        ctx.bufferData(
            ctx.ELEMENT_ARRAY_BUFFER, edata, ctx.DYNAMIC_DRAW);
        ctx.drawElements(
            ctx.TRIANGLE_STRIP, 2*(sd2+1), ctx.UNSIGNED_SHORT, 0);
    }

    // Disable attributes
    for (var i in attriblocs)
        if (attriblocs[i] != -1)
            ctx.disableVertexAttribArray(attriblocs[i]);

    // Delete buffer objects
    for (i in buffs)
        ctx.deleteBuffer(buffs[i]);
    ctx.deleteBuffer(ebuff);
}


// normalMatrixAsFloat32Array_internal_
// Computer normal matrix (as Float32Array of 9 items) based on given
// model/view matrix.
function normalMatrixAsFloat32Array_internal_(mvMat)
{
    // Find transpose of inverse of model/view
    var inverseMat = new J3DIMatrix4();
    inverseMat.load(mvMat);
    inverseMat.invert();

    // Normal matrix is transpose of upper-left 3x3 of above
    var nm4 = inverseMat.getAsFloat32Array();
    var nm3 = new Float32Array([
        nm4[0], nm4[4], nm4[8],
        nm4[1], nm4[5], nm4[9],
        nm4[2], nm4[6], nm4[10]]);

    return nm3;
}

// sendMatrices
// Assumes ctx has members pMatrix, mvMatrix, tMatrix that are
// J3DIMatrix4 objects. Sends the three matrices to current active
// shaders: uniform mat4 variables with names projectionMatrix,
// modelViewMatrix, and textureMatrix -- if these exist in the shaders.
// Computes normal matrix and sends this to uniform mat3 variable with
// name normalMatrix -- if this exists in the shaders.
function sendMatrices(ctx)
{
    var shaderProgram =
        getProgram_internal_(ctx, arguments.callee.name);
    if (!shaderProgram)
        return;

    var loc;  // Location of vars in shaders
    var mvMatrixErrorPrinted = false;
              // True if no-mvMatrix-member error msg has been printed

    // Projection matrix
    loc = ctx.getUniformLocation(shaderProgram, 'projectionMatrix');
    if (loc != -1)
    {
        if (!ctx.pMatrix)
            errOut(arguments.callee.name + ': ' +
                   'No pMatrix member in WebGL context');
        else
            ctx.pMatrix.setUniform(ctx, loc, false);
    }

    // Model/view matrix
    loc = ctx.getUniformLocation(shaderProgram, 'modelViewMatrix');
    if (loc != -1)
    {
        if (!ctx.mvMatrix)
        {
            errOut(arguments.callee.name + ': ' +
                   'No mvMatrix member in WebGL context');
            mvMatrixErrorPrinted = true;
        }
        else
            ctx.mvMatrix.setUniform(ctx, loc, false);
    }

    // Normal matrix
    loc = ctx.getUniformLocation(shaderProgram, 'normalMatrix');
    if (loc != -1)
    {
        if (!ctx.mvMatrix && !mvMatrixErrorPrinted)
            errOut(arguments.callee.name + ': ' +
                   'No mvMatrix member in WebGL context');
        else
        {
            // Compute normal matrix
            var nMatArray = normalMatrixAsFloat32Array_internal_(
                ctx.mvMatrix);

            // Send it to the shaders
            ctx.uniformMatrix3fv(loc, false, nMatArray);
        }
    }

    // Texture matrix
    loc = ctx.getUniformLocation(shaderProgram, 'textureMatrix');
    if (loc != -1)
    {
        if (!ctx.tMatrix)
            errOut(arguments.callee.name + ': ' +
                   'No tMatrix member in WebGL context');
        else
            ctx.tMatrix.setUniform(ctx, loc, false);
    }
}


// makeShaderObject_internal_
// Given WebGL context, string holding GLSL source for shader, and
// Boolean indicating vertex (true) or fragment (false) shader,
// returns shader object, or null on failure.
function makeShaderObject_internal_(ctx, shaderText, isVert)
{
    var shaderType = isVert ? ctx.VERTEX_SHADER : ctx.FRAGMENT_SHADER;
    var shaderStr = isVert ? 'vertex' : 'fragment';

    // Create shader object
    var shader = ctx.createShader(shaderType);
    if (!shader)
    {
        errOut(arguments.callee.name + ': ' +
               'Cannot create ' + shaderStr + ' shader object');
        return null;
    }

    // Load, compile shader source
    ctx.shaderSource(shader, shaderText);
    ctx.compileShader(shader);

    // Check compile status
    var compiled = ctx.getShaderParameter(shader, ctx.COMPILE_STATUS);
    if (!compiled)
    {
        var error = ctx.getShaderInfoLog(shader);
        errOut(arguments.callee.name + ': ' +
               'Error compiling ' + shaderStr + ' shader, msg =');
        errOut(error);
        ctx.deleteShader(shader);
        return null;
    }

    return shader;
}


// makeProgramObject
// Given WebGL context, strings holding source for GLSL vertex,
// fragment shaders, returns program object, or null on failure.
function makeProgramObject(ctx, vShaderText, fShaderText)
{
    if (!checkContext_internal_(ctx, arguments.callee.name))
        return null;

    // Make shaders
    var vShader = makeShaderObject_internal_(ctx, vShaderText, true);
    if (!vShader)
        return null;
    var fShader = makeShaderObject_internal_(ctx, fShaderText, false);
    if (!fShader)
        return null;

    // Make program object
    var shaderProgram = ctx.createProgram();
    if (!shaderProgram)
        return null;

    // Attach shaders to program object
    ctx.attachShader(shaderProgram, vShader);
    ctx.attachShader(shaderProgram, fShader);

    // Link shaders
    ctx.linkProgram(shaderProgram);

    // Check link status
    var linked =
        ctx.getProgramParameter(shaderProgram, ctx.LINK_STATUS);
    if (!linked)
    {
        var error = ctx.getProgramInfoLog(shaderProgram);
        errOut(arguments.callee.name + ': ' +
               'shader linking error, msg =');
        errOut(error);

        ctx.deleteProgram(shaderProgram);
        ctx.deleteProgram(fShader);
        ctx.deleteProgram(vShader);

        return null;
    }

    return shaderProgram;
}


// makeProgramObjectFromIds
// Given WebGL context, strings holding element Ids for vertex, fragment
// shaders, returns program object, or null on failure.
function makeProgramObjectFromIds(ctx, vShaderId, fShaderId)
{
    if (!checkContext_internal_(ctx, arguments.callee.name))
        return null;
    if (typeof vShaderId != 'string')
    {
        errOut(arguments.callee.name + ': ' +
               'Vertex shader Id not given');
        return null;
    }
    if (typeof fShaderId != 'string')
    {
        errOut(arguments.callee.name + ': ' +
               'Fragment shader Id not given');
        return null;
    }

    var vShaderScript = document.getElementById(vShaderId);
    if (!vShaderScript)
    {
        errOut(arguments.callee.name + ': ' +
               'Vertex shader script [' + vShaderId + '] not found');
        return null;
    }
    if (vShaderScript.nodeName.toLowerCase() != 'script' ||
        !vShaderScript.type ||
        vShaderScript.type != 'x-shader/x-vertex')
    {
        errOut(arguments.callee.name + ': ' +
               'Script [' + vShaderId + '] is not a vertex shader');
        return null;
    }

    var fShaderScript = document.getElementById(fShaderId);
    if (!fShaderScript)
    {
        errOut(arguments.callee.name + ': ' +
               'Vertex shader script [' + fShaderId + '] not found');
        return null;
    }
    if (fShaderScript.nodeName.toLowerCase() != 'script' ||
        !fShaderScript.type ||
        fShaderScript.type != 'x-shader/x-fragment')
    {
        errOut(arguments.callee.name + ': ' +
               'Script [' + fShaderId + '] is not a fragment shader');
        return null;
    }

    return makeProgramObject(ctx,
                             vShaderScript.text, fShaderScript.text);
}


// getCanvas
// Given Id of canvas element, returns canvas object, or null if
// failure.
function getCanvas(canvasId)
{
    if (typeof canvasId != 'string')
    {
        errOut(arguments.callee.name + ': ' +
               'Canvas Id not given');
        return null;
    }

    var canvas = document.getElementById(canvasId);
    if (!canvas)
    {
        errOut(arguments.callee.name + ': ' +
               'Canvas [' + canvasId + '] not found');
        return null;
    }
    if (canvas.nodeName.toLowerCase() != 'canvas')
    {
        errOut(arguments.callee.name + ': ' +
               'Elements [' + canvasId + '] is not a canvas');
        return null;
    }

    return canvas;
}


// pushPMatrix, pushMvMatrix, pushTMatrix
// In the given context, push the appropriate matrix (pMatrix, mvMatrix,
// or tMatrix) on the associated stack.
// Assumes context has members pMatrix and pStack_internal_ (and similar
// members for mv, t).
function pushPMatrix(ctx)
{
    ctx.pStack_internal_.push(new J3DIMatrix4(ctx.pMatrix));
}

function pushMvMatrix(ctx)
{
    ctx.mvStack_internal_.push(new J3DIMatrix4(ctx.mvMatrix));
}

function pushTMatrix(ctx)
{
    ctx.tStack_internal_.push(new J3DIMatrix4(ctx.tMatrix));
}


// popPMatrix, popMvMatrix, popTMatrix
// In the given context, pop the appropriate matrix (pMatrix, mvMatrix,
// or tMatrix) off the associated stack. Gives error message if called
// with empty stack.
// Assumes context has members pMatrix and pStack_internal_ (and similar
// members for mv, t).
function popPMatrix(ctx)
{
    if (ctx.pStack_internal_.length == 0)
    {
        errOut(arguments.callee.name + ': ' +
               'Projection matrix stack popped when empty');
        return;
    }
    ctx.pMatrix.load(ctx.pStack_internal_.pop());
}

function popMvMatrix(ctx)
{
    if (ctx.mvStack_internal_.length == 0)
    {
        errOut(arguments.callee.name + ': ' +
               'Model/view matrix stack popped when empty');
        return;
    }
    ctx.mvMatrix.load(ctx.mvStack_internal_.pop());
}

function popTMatrix(ctx)
{
    if (ctx.tStack_internal_.length == 0)
    {
        errOut(arguments.callee.name + ': ' +
               'Texture matrix stack popped when empty');
        return;
    }
    ctx.tMatrix.load(ctx.tStack_internal_.pop());
}


// getGlContext
// Given canvas object, returns WebGL context, or null on failure.
// Returned context has the following members added:
// - J3DIMatrix4 objects:
//   - mvMatrix, pMatrix, tMatrix
// - Array objects (accessed via push, pop functions):
//   - pStack_internal_, mvStack_internal_, tStack_internal_
function getGlContext(canvas)
{
    if (typeof canvas != 'object' ||
        !('nodeName' in canvas) ||
        typeof canvas.nodeName != 'string' ||
        canvas.nodeName.toLowerCase() != 'canvas')
    {
        errOut(arguments.callee.name + ': ' +
               'Canvas object not given');
        return null;
    }

    var ctx = null;
    try
    {
        // Below based on webgl-utils.js from Google
        var webglnames = ['webgl',
                          'experimental-webgl',
                          'webkit-3d',
                          'moz-webgl'];
        for (var i in webglnames)
        {
            ctx = canvas.getContext(webglnames[i]);
            if (ctx)
                break;
        }
    }
    catch (e) {}

    if (!ctx)
    {
        var errmsg = 'Could not initialize WebGL';
        document.write(errmsg);
        errOut(arguments.callee.name + ': ' +
               errmsg);
        return null;
    }

    // Create projection, model/view, texture matrices
    ctx.pMatrix = new J3DIMatrix4();
    ctx.pMatrix.makeIdentity();
    ctx.mvMatrix = new J3DIMatrix4();
    ctx.mvMatrix.makeIdentity();
    ctx.tMatrix = new J3DIMatrix4();
    ctx.tMatrix.makeIdentity();

    // Create projection, model/view, texture stacks
    ctx.pStack_internal_ = new Array();
    ctx.mvStack_internal_ = new Array();
    ctx.tStack_internal_ = new Array();

    return ctx;
}


// requestAnimFrame_internal_
// Returns function window.requestAnimationFrame, or, if that does not
// exist, a function that does much the same thing.
// Based on webgl-utils.js from Google.
requestAnimFrame_internal_ = (function()
{
    return window.requestAnimationFrame ||
           window.webkitRequestAnimationFrame ||
           window.mozRequestAnimationFrame ||
           window.oRequestAnimationFrame ||
           window.msRequestAnimationFrame ||
           function(callback, element)
           {
                 window.setTimeout(callback, 1000/60);
           };
})();


// animate
// Given function to call, does repeated animation frames, calling the
// given function at each frame.
function animate(func)
{
    // Are we given a function?
    if (typeof func != 'function')
    {
        errOut(arguments.callee.name + ': ' +
               'Argument is not a function');
        return;
    }

    // Set up next frame: call this function with same argument.
    var thisfunc = arguments.callee;
    requestAnimFrame_internal_(function() { thisfunc(func); });

    // Call given function
    func();
}


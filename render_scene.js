
// code taken from: https://hashrocket.com/blog/posts/using-tiled-and-canvas-to-render-game-screens
$(function() {
    var width = 0; var height = 0; var tilesize = 0;

    var bgLayerCanvas = $("#backgroundLayer")[0];
    bgLayerCanvas.width = window.innerWidth;
    bgLayerCanvas.height = window.innerHeight;
    var ctx_bg = bgLayerCanvas.getContext("2d");

    var robotsLayerCanvas = $("#robotsLayer")[0];
    robotsLayerCanvas.width = bgLayerCanvas.width;
    robotsLayerCanvas.height = bgLayerCanvas.height;
    var ctx_robots = robotsLayerCanvas.getContext("2d");
    ctx_robots.font = "12px Arial";

    var cloudsLayerCanvas = $("#cloudsLayer")[0];
    var ctx_clouds = cloudsLayerCanvas.getContext("2d");
    cloudsLayerCanvas.width = bgLayerCanvas.width;
    cloudsLayerCanvas.height = bgLayerCanvas.height;
    ctx_clouds.fillStyle = "black";
    ctx_clouds.fillRect(0, 0, cloudsLayerCanvas.width, cloudsLayerCanvas.height);

    // track drag and zoom on ctx_cloud
    trackTransforms(ctx_cloud);


    var mapReady = new Event('mapready');

    var robotImg = new Image();
    robotImg.src = "res/wall-e.png";

    var cloudImg = new Image();
    cloudImg.src = "res/cloud.png";


    var robots = {};

    var showClouds = function() {
        ctx_clouds.fillStyle = "rgba(0, 0, 0, .01)";
        ctx_clouds.globalCompositeOperation = 'source-over';
        ctx_clouds.fillRect(0, 0, cloudsLayerCanvas.width, cloudsLayerCanvas.height);
        ctx_clouds.globalCompositeOperation = 'destination-out';

        for (var name in robots) {
            x = robots[name][0] * tilesize + tilesize/2;
            y = robots[name][1] * tilesize + tilesize/2;
            angle = Math.random() * Math.PI * 2;
            ctx_clouds.save();
            ctx_clouds.translate(x,y);
            ctx_clouds.rotate(angle);
            ctx_clouds.drawImage(cloudImg, - cloudImg.width/2, - cloudImg.height/2);
            ctx_clouds.restore();
        }
    };

    var showRobots = function() {
        ctx_robots.clearRect(0, 0, robotsLayerCanvas.width, robotsLayerCanvas.height);
        for (var name in robots) {
            x = robots[name][0] * tilesize;
            y = robots[name][1] * tilesize;
            ctx_robots.drawImage(robotImg, x, y);
            ctx_robots.beginPath();
            textW = ctx_robots.measureText(name).width;
            ctx_robots.rect(x - (textW - tilesize)/2, y + tilesize, textW + 2, 16);
            ctx_robots.fillStyle = "rgba(255, 255, 255, .4)";
            ctx_robots.fill();
            ctx_robots.fillStyle = "rgba(0,0,0,1)";
            ctx_robots.fillText(name, x - (textW-tilesize)/2 + 2, y + tilesize + 12 + 1);
        }
    };

    var scene = {
        layers: [],
        renderLayer: function(layer) {
            // data: [array of tiles, 1-based, position of sprite from top-left]
            // height: integer, height in number of sprites
            // name: "string", internal name of layer
            // opacity: integer
            // type: "string", layer type (tile, object)
            // visible: boolean
            // width: integer, width in number of sprites
            // x: integer, starting x position
            // y: integer, starting y position
            if (layer.type !== "tilelayer" || !layer.opacity) { return; }
            var s = document.createElement('canvas');
            s.width = 3200;
            s.height = 3200;
            size = scene.data.tilewidth;
            s = s.getContext("2d");
            if (scene.layers.length < scene.data.layers.length) {
                width = layer.width
                height = layer.height
                tilesize = scene.data.tilewidth
                if (layer.name == "Route") {// this is the route layer for our robots!! Send that to the backend
                    console.log("Sending the route map to the backend");
                    $.post('/api?map', 
                        JSON.stringify(
                            {"width": layer.width,
                                "height": layer.height,
                                "data": layer.data})
                    );
                }

                layer.data.forEach(function(tile_idx, i) {
                    if (!tile_idx) { return; }
                    var img_x, img_y, s_x, s_y,
                        tile = scene.data.tilesets[0];
                    tile_idx--;
                    img_x = (tile_idx % (tile.imagewidth / size)) * size;
                    img_y = ~~(tile_idx / (tile.imagewidth / size)) * size;
                    s_x = (i % layer.width) * size;
                    s_y = ~~(i / layer.width) * size;
                    s.drawImage(scene.tileset, img_x, img_y, size, size,
                        s_x, s_y, size, size);
                });
                scene.layers.push(s.canvas.toDataURL());
                ctx_bg.drawImage(s.canvas, 0, 0);
            }
            else {
                scene.layers.forEach(function(src) {
                    var i = $("<img />", { src: src })[0];
                    ctx_bg.drawImage(i, 0, 0);
                });
            }
        },
        renderLayers: function(layers) {
            layers = $.isArray(layers) ? layers : this.data.layers;
            layers.forEach(this.renderLayer);
            document.dispatchEvent(mapReady);
        },
        loadTileset: function(json) {
            this.data = json;
            this.tileset = $("<img />", { src: json.tilesets[0].image })[0]
            this.tileset.onload = $.proxy(this.renderLayers, this);
        },
        load: function(name) {
            // note: appending timestamp to force a fresh reload everytime (bypass cache)
            return $.getJSON("res/" + name + ".json?t=" + (new Date()).getTime()).done($.proxy(this.loadTileset, this));
        }
    };

    //  scene.load("mountain");
    scene.load("countryside");

    setInterval(showClouds,200);

    document.addEventListener('mapready', function(e) {
        setInterval(function() {
            $.get('/api?get_robots', function(data) {

                newrobots = JSON.parse(data);
                if (newrobots != robots) {
                    robots = newrobots;
                    showRobots();
                }
            });
        }, 200);
    }, false);

    function redraw(){

          // Clear the entire canvas
          var p1 = ctx_bg.transformedPoint(0,0);
          var p2 = ctx_bg.transformedPoint(bgLayerCanvas.width,bgLayerCanvas.height);
          ctx_bg.clearRect(p1.x,p1.y,p2.x-p1.x,p2.y-p1.y);

          ctx_bg.save();
          ctx_bg.setTransform(1,0,0,1,0,0);
          ctx_bg.clearRect(0,0,bgLayerCanvas.width,bgLayerCanvas.height);
          ctx_bg.restore();

          // redraw the tile layers
          var img = new Image;
          for (var layer_idx in scene.layers) {
              img.src = scene.layers[layer_idx];
              ctx_bg.drawImage(img, 0, 0);
          }

          //scene.data.layers.forEach(scene.renderLayer);

        }

      var lastX=bgLayerCanvas.width/2, lastY=bgLayerCanvas.height/2;

      var dragStart,dragged;

      cloudsLayerCanvas.addEventListener('mousedown',function(evt){
          document.body.style.mozUserSelect = document.body.style.webkitUserSelect = document.body.style.userSelect = 'none';
          lastX = evt.offsetX || (evt.pageX - bgLayerCanvas.offsetLeft);
          lastY = evt.offsetY || (evt.pageY - bgLayerCanvas.offsetTop);
          dragStart = ctx_bg.transformedPoint(lastX,lastY);
          dragged = false;
      },false);

      cloudsLayerCanvas.addEventListener('mousemove',function(evt){
          lastX = evt.offsetX || (evt.pageX - bgLayerCanvas.offsetLeft);
          lastY = evt.offsetY || (evt.pageY - bgLayerCanvas.offsetTop);
          dragged = true;
          if (dragStart){
            var pt = ctx_bg.transformedPoint(lastX,lastY);
            ctx_bg.translate(pt.x-dragStart.x,pt.y-dragStart.y);
            redraw();
                }
      },false);

      cloudsLayerCanvas.addEventListener('mouseup',function(evt){
          dragStart = null;
          if (!dragged) zoom(evt.shiftKey ? -1 : 1 );
      },false);

      var scaleFactor = 1.1;

      var zoom = function(clicks){
          var pt = ctx_bg.transformedPoint(lastX,lastY);
          ctx_bg.translate(pt.x,pt.y);
          ctx_robots.translate(pt.x,pt.y);
          ctx_clouds.translate(pt.x,pt.y);
          var factor = Math.pow(scaleFactor,clicks);
          ctx_bg.scale(factor,factor);
          ctx_robots.scale(factor,factor);
          ctx_clouds.scale(factor,factor);
          ctx_bg.translate(-pt.x,-pt.y);
          ctx_robots.translate(-pt.x,-pt.y);
          ctx_clouds.translate(-pt.x,-pt.y);
          redraw();
      }

      var handleScroll = function(evt){
          var delta = evt.wheelDelta ? evt.wheelDelta/40 : evt.detail ? -evt.detail : 0;
          if (delta) zoom(delta);
          return evt.preventDefault() && false;
      };

      cloudsLayerCanvas.addEventListener('DOMMouseScroll',handleScroll,false);
      cloudsLayerCanvas.addEventListener('mousewheel',handleScroll,false);
});


// Taken from https://codepen.io/techslides/pen/zowLd
// Adds ctx.getTransform() - returns an SVGMatrix
// Adds ctx.transformedPoint(x,y) - returns an SVGPoint
function trackTransforms(ctx){
    var svg = document.createElementNS("http://www.w3.org/2000/svg",'svg');
    var xform = svg.createSVGMatrix();
    ctx.getTransform = function(){ return xform; };

    var savedTransforms = [];
    var save = ctx.save;
    ctx.save = function(){
        savedTransforms.push(xform.translate(0,0));
        return save.call(ctx);
    };

    var restore = ctx.restore;
    ctx.restore = function(){
        xform = savedTransforms.pop();
        return restore.call(ctx);
    };

    var scale = ctx.scale;
    ctx.scale = function(sx,sy){
        xform = xform.scaleNonUniform(sx,sy);
        return scale.call(ctx,sx,sy);
    };

    var rotate = ctx.rotate;
    ctx.rotate = function(radians){
        xform = xform.rotate(radians*180/Math.PI);
        return rotate.call(ctx,radians);
    };

    var translate = ctx.translate;
    ctx.translate = function(dx,dy){
        xform = xform.translate(dx,dy);
        return translate.call(ctx,dx,dy);
    };

    var transform = ctx.transform;
    ctx.transform = function(a,b,c,d,e,f){
        var m2 = svg.createSVGMatrix();
        m2.a=a; m2.b=b; m2.c=c; m2.d=d; m2.e=e; m2.f=f;
        xform = xform.multiply(m2);
        return transform.call(ctx,a,b,c,d,e,f);
    };

    var setTransform = ctx.setTransform;
    ctx.setTransform = function(a,b,c,d,e,f){
        xform.a = a;
        xform.b = b;
        xform.c = c;
        xform.d = d;
        xform.e = e;
        xform.f = f;
        return setTransform.call(ctx,a,b,c,d,e,f);
    };

    var pt  = svg.createSVGPoint();
    ctx.transformedPoint = function(x,y){
        pt.x=x; pt.y=y;
        return pt.matrixTransform(xform.inverse());
    }


}

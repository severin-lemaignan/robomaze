
// code taken from: https://hashrocket.com/blog/posts/using-tiled-and-canvas-to-render-game-screens
$(function() {
    var width = 0; var height = 0; var tilesize = 0;

    var c = $("#backgroundLayer")[0].getContext("2d");
    var robotsLayerCanvas = $("#robotsLayer")[0];
    var c_robots = robotsLayerCanvas.getContext("2d");
    c_robots.font = "12px Arial";
    var cloudsLayerCanvas = $("#cloudsLayer")[0];
    var c_clouds = cloudsLayerCanvas.getContext("2d");


    var mapReady = new Event('mapready');

    var robotImg = new Image();
    robotImg.src = "res/wall-e.png";

    var cloudImg = new Image();
    cloudImg.src = "res/cloud.png";


    var robots = {};

    var showClouds = function() {
        c_clouds.fillStyle = "rgba(0, 0, 0, .1)";
        c_clouds.globalCompositeOperation = 'source-over';
        c_clouds.fillRect(0, 0, cloudsLayerCanvas.width, cloudsLayerCanvas.height);
        c_clouds.globalCompositeOperation = 'destination-out';

        for (var name in robots) {
            x = robots[name][0] * tilesize + tilesize/2;
            y = robots[name][1] * tilesize + tilesize/2;
            c_clouds.drawImage(cloudImg, x - cloudImg.width, y - cloudImg.height);
        }
    };

    var showRobots = function() {
        c_robots.clearRect(0, 0, robotsLayerCanvas.width, robotsLayerCanvas.height);
        for (var name in robots) {
            x = robots[name][0] * tilesize;
            y = robots[name][1] * tilesize;
            c_robots.drawImage(robotImg, x, y);
            c_robots.beginPath();
            textW = c_robots.measureText(name).width;
            c_robots.rect(x - (textW - tilesize)/2, y + tilesize, textW + 2, 16);
            c_robots.fillStyle = "rgba(255, 255, 255, .4)";
            c_robots.fill();
            c_robots.fillStyle = "rgba(0,0,0,1)";
            c_robots.fillText(name, x - (textW-tilesize)/2 + 2, y + tilesize + 12 + 1);
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
            var s = c.canvas.cloneNode(),
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
                c.drawImage(s.canvas, 0, 0);
            }
            else {
                scene.layers.forEach(function(src) {
                    var i = $("<img />", { src: src })[0];
                    c.drawImage(i, 0, 0);
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
            return $.getJSON("res/" + name + ".json").done($.proxy(this.loadTileset, this));
        }
    };

    //  scene.load("mountain");
    scene.load("countryside");

    document.addEventListener('mapready', function(e) {
        setInterval(function() {
            $.get('/api?get_robots', function(data) {
                robots = JSON.parse(data);
                showRobots();
                showClouds();

            });
        }, 1000);
    }, false);
});

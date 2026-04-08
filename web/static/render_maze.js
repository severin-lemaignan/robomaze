(function () {
    "use strict";

    const canvas = document.getElementById("maze");
    const ctx = canvas.getContext("2d");
    const robotListEl = document.getElementById("robot_list");

    let mapData = null;       // from /api/render_map
    let mazeWidth = 0;
    let mazeHeight = 0;
    let tilesetImg = null;
    let robotImg = null;
    let targetImg = null;
    let goal = null;
    let robots = {};

    // Tileset metadata (filled from server)
    let ts = {};  // tile_size, cols, positions, floor_positions, background_positions

    // Pre-rendered maze (offscreen canvas, drawn once)
    let mazeCanvas = null;

    // Pan/zoom state
    let transform = { x: 0, y: 0, scale: 1 };
    let dragStart = null;
    let lastMouse = { x: 0, y: 0 };

    // --- Seeded random for consistent floor pattern ---
    let _seed = 1;
    function seededRandom() {
        _seed = (_seed * 16807 + 0) % 2147483647;
        return (_seed - 1) / 2147483646;
    }

    // --- Color from name (deterministic) ---
    function nameHash(str) {
        let hash = 0;
        for (let i = 0; i < str.length; i++) {
            hash = ((hash << 5) - hash) + str.charCodeAt(i);
            hash |= 0;
        }
        return hash;
    }

    function nameColor(name) {
        const letters = "456789ABCDEF";
        let color = "#";
        const h = nameHash(name);
        for (let i = 0; i < 6; i++) {
            const x = Math.sin(h + i) * 10000;
            color += letters[Math.floor((x - Math.floor(x)) * 12)];
        }
        return color;
    }

    // --- Draw a tile from the tileset onto a context ---
    function drawTile(destCtx, tileType, dx, dy, displaySize) {
        const posKey = String(tileType);
        const pos = ts.positions[posKey];
        if (!pos) return;
        const [row, col] = pos;
        const s = ts.tile_size;
        destCtx.drawImage(tilesetImg, col * s, row * s, s, s, dx, dy, displaySize, displaySize);
    }

    function drawTileFromPos(destCtx, row, col, dx, dy, displaySize) {
        const s = ts.tile_size;
        destCtx.drawImage(tilesetImg, col * s, row * s, s, s, dx, dy, displaySize, displaySize);
    }

    // --- Build the offscreen maze canvas (called once after data loads) ---
    function buildMaze() {
        const displaySize = ts.tile_size;  // render at native tileset resolution
        mazeCanvas = document.createElement("canvas");
        mazeCanvas.width = mazeWidth * displaySize;
        mazeCanvas.height = mazeHeight * displaySize;
        const mctx = mazeCanvas.getContext("2d");

        // Background pass
        if (ts.background_positions && ts.background_positions.length > 0) {
            const [bgRow, bgCol] = ts.background_positions[0];
            for (let y = 0; y < mazeHeight; y++) {
                for (let x = 0; x < mazeWidth; x++) {
                    drawTileFromPos(mctx, bgRow, bgCol, x * displaySize, y * displaySize, displaySize);
                }
            }
        } else {
            mctx.fillStyle = "#8cb2fa";
            mctx.fillRect(0, 0, mazeCanvas.width, mazeCanvas.height);
        }

        // Tile pass (floor + walls)
        _seed = 1;  // reset for consistent floor pattern
        const tiles = mapData.tiles;
        for (let y = 0; y < mazeHeight; y++) {
            for (let x = 0; x < mazeWidth; x++) {
                const idx = x + y * mazeWidth;
                const tileType = tiles[idx];
                const dx = x * displaySize;
                const dy = y * displaySize;

                if (tileType === 1) {
                    // FLOOR — pick a random variant
                    const floorPos = ts.floor_positions;
                    const pick = Math.floor(seededRandom() * floorPos.length);
                    const [fr, fc] = floorPos[pick];
                    drawTileFromPos(mctx, fr, fc, dx, dy, displaySize);
                } else if (tileType > 0) {
                    drawTile(mctx, tileType, dx, dy, displaySize);
                }
                // tileType === -1: fully enclosed wall — leave background visible
            }
        }

        // CCV overlay pass
        if (mapData.ccv_overlays) {
            for (const entry of mapData.ccv_overlays) {
                const x = entry.idx % mazeWidth;
                const y = Math.floor(entry.idx / mazeWidth);
                const dx = x * displaySize;
                const dy = y * displaySize;
                for (const ccvType of entry.types) {
                    drawTile(mctx, ccvType, dx, dy, displaySize);
                }
            }
        }
    }

    // --- Main render loop ---
    function render() {
        // Resize canvas to fill container
        const wrap = canvas.parentElement;
        canvas.width = wrap.clientWidth;
        canvas.height = wrap.clientHeight;

        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#111";
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        if (!mazeCanvas) return;

        // Apply pan/zoom
        ctx.setTransform(transform.scale, 0, 0, transform.scale, transform.x, transform.y);

        // Draw pre-rendered maze
        ctx.drawImage(mazeCanvas, 0, 0);

        const displaySize = ts.tile_size;

        // Draw goal
        if (goal && targetImg && targetImg.complete) {
            const gx = goal[0];
            const gy = mazeHeight - goal[1] - 1;
            const targetW = displaySize * 0.75;
            const targetH = targetW * (targetImg.naturalHeight / targetImg.naturalWidth);
            const tx = gx * displaySize + (displaySize - targetW) / 2;
            const ty = gy * displaySize + (displaySize - targetW) / 2 - (targetH - targetW);
            ctx.drawImage(targetImg, tx, ty, targetW, targetH);
        }

        // Draw robots
        for (const name in robots) {
            const r = robots[name];
            const rx = r.pos[0];
            const ry = mazeHeight - r.pos[1] - 1;
            const px = rx * displaySize;
            const py = ry * displaySize;

            // Robot sprite
            if (robotImg && robotImg.complete) {
                ctx.drawImage(robotImg, px, py, displaySize, displaySize);
            } else {
                ctx.fillStyle = nameColor(name);
                ctx.fillRect(px + 4, py + 4, displaySize - 8, displaySize - 8);
            }

            // Name label
            ctx.font = `${Math.max(12, displaySize / 5)}px sans-serif`;
            const textW = ctx.measureText(name).width;
            const labelX = px + (displaySize - textW) / 2;
            const labelY = py + displaySize + displaySize / 4;
            ctx.fillStyle = nameColor(name);
            ctx.fillRect(labelX - 2, labelY - displaySize / 5, textW + 4, displaySize / 4 + 2);
            ctx.fillStyle = "#000";
            ctx.fillText(name, labelX, labelY);
        }

        requestAnimationFrame(render);
    }

    // --- Robot list sidebar ---
    function updateRobotList() {
        const names = Object.keys(robots);
        if (names.length === 0) {
            robotListEl.innerHTML = "";
            return;
        }
        // Rebuild if count changed
        if (robotListEl.children.length !== names.length) {
            robotListEl.innerHTML = "";
            for (const name of names) {
                const box = document.createElement("span");
                box.className = "robotbox";
                box.id = name + "_box";
                box.style.backgroundColor = nameColor(name);
                const r = robots[name];
                const ageStr = mmss(r.age) + (r.finished ? " ✓" : "");
                box.innerHTML =
                    `<b>${name}</b> ` +
                    `<span class="life">${r.life}</span> ` +
                    `<span class="age">${ageStr}</span>`;
                robotListEl.appendChild(box);
            }
        } else {
            for (const name of names) {
                const box = document.getElementById(name + "_box");
                if (box) {
                    const r = robots[name];
                    box.querySelector(".life").textContent = r.life;
                    box.querySelector(".age").textContent = mmss(r.age) + (r.finished ? " ✓" : "");
                }
            }
        }
    }

    function mmss(secs) {
        const m = String(Math.floor(secs / 60)).padStart(2, "0");
        const s = String(Math.floor(secs % 60)).padStart(2, "0");
        return m + ":" + s;
    }

    // --- Polling ---
    function pollRobots() {
        fetch("/api/robots")
            .then(r => r.json())
            .then(data => {
                robots = data;
                updateRobotList();
            })
            .catch(() => {});
    }

    // --- Pan/zoom ---
    canvas.addEventListener("mousedown", function (e) {
        lastMouse = { x: e.offsetX, y: e.offsetY };
        dragStart = { x: e.offsetX, y: e.offsetY };
    });

    canvas.addEventListener("mousemove", function (e) {
        if (dragStart) {
            const dx = e.offsetX - lastMouse.x;
            const dy = e.offsetY - lastMouse.y;
            transform.x += dx;
            transform.y += dy;
            lastMouse = { x: e.offsetX, y: e.offsetY };
        }
    });

    canvas.addEventListener("mouseup", function () {
        dragStart = null;
    });

    canvas.addEventListener("mouseleave", function () {
        dragStart = null;
    });

    canvas.addEventListener("wheel", function (e) {
        e.preventDefault();
        const zoom = e.deltaY < 0 ? 1.1 : 1 / 1.1;
        const mx = e.offsetX;
        const my = e.offsetY;
        transform.x = mx - zoom * (mx - transform.x);
        transform.y = my - zoom * (my - transform.y);
        transform.scale *= zoom;
    }, { passive: false });

    // --- Initial fit ---
    function fitToScreen() {
        if (!mazeCanvas) return;
        const wrap = canvas.parentElement;
        const scaleX = wrap.clientWidth / mazeCanvas.width;
        const scaleY = wrap.clientHeight / mazeCanvas.height;
        transform.scale = Math.min(scaleX, scaleY) * 0.95;
        transform.x = (wrap.clientWidth - mazeCanvas.width * transform.scale) / 2;
        transform.y = (wrap.clientHeight - mazeCanvas.height * transform.scale) / 2;
    }

    // --- Bootstrap ---
    async function init() {
        // Load render map data
        const renderRes = await fetch("/api/render_map");
        mapData = await renderRes.json();
        mazeWidth = mapData.width;
        mazeHeight = mapData.height;
        ts = mapData.tileset;

        // Load map info (for goal)
        const mapRes = await fetch("/api/map");
        const mapInfo = await mapRes.json();
        goal = mapInfo.goal;

        // Load images
        tilesetImg = new Image();
        robotImg = new Image();
        targetImg = new Image();
        tilesetImg.src = "/res/tileset.png";
        robotImg.src = "/res/walle_top.png";
        targetImg.src = "/res/target.png";

        await new Promise(resolve => { tilesetImg.onload = resolve; });

        buildMaze();
        fitToScreen();

        // Start render loop and polling
        requestAnimationFrame(render);
        setInterval(pollRobots, 200);
    }

    init();
})();

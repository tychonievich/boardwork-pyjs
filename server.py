from aiohttp import web, WSMsgType
import asyncio
import pdfmaker
from pathlib import Path
from sys import path

routes = web.RouteTableDef()
websockets = set()
lurkers = set()

@routes.get('/')
async def websocket_handler(request):
    return web.FileResponse(Path(path[0], 'web/index.html'))

@routes.get('/{name:.*(je?pg|png|gif|avif|webp)}')
async def websocket_handler(request):
    """Annoying: routes.static doesn't allow two routes for same prefix,
    but I want both source's web (for js, css, etc) and cwd (for images in slides).
    This is my workaround."""
    name = request.match_info['name']
    if Path(name).exists():
        return web.FileResponse(name)
    return web.FileResponse(Path(path[0], 'web',name))


routes.static('/',Path(path[0], 'web'))

@routes.get('/ws')
async def websocket_handler(request):
    """Dumps every entering message to a file"""
    from datetime import datetime

    ws = web.WebSocketResponse()
    await ws.prepare(request)
    websockets.add(ws)
    try:
        fname = datetime.now().strftime('%Y-%m-%d.json')
        if Path(fname).exists():
            await ws.send_str(open(fname, 'r').read())
        with open(fname, 'a') as dst:
            print('+ws connected to',fname)
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    s = msg.data
                    if '"live":true' not in s:
                        print(msg.data, file=dst, flush=True)
                    else:
                        pub(msg.data)
            print('-ws disconnected',fname)
        pdfmaker.log2pdf(fname, fname[:fname.rfind('.')]+'.pdf')
    finally:
        websockets.discard(ws)

    return ws

@routes.get('/lurkws')
async def websocket_handler(request):
    """Dumps every entering message to a file"""
    from datetime import datetime

    ws = web.WebSocketResponse()
    await ws.prepare(request)
    websockets.add(ws)
    lurkers.add(ws)
    try:
        fname = datetime.now().strftime('%Y-%m-%d.json')
        if Path(fname).exists():
            await ws.send_str(open(fname, 'r').read())
        print('+lurker',id(ws),'connected to',fname)
        async for msg in ws:
            pass
        print('-lurker',id(ws),'disconnected')
    finally:
        lurkers.discard(ws)
        websockets.discard(ws)

    return ws

def pub(msg):
    for lurker in lurkers:
        asyncio.create_task(lurker.send_str(msg))


async def close_sockets(app):
    for ws in tuple(websockets):
        await ws.close()

if __name__ == '__main__':
    app = web.Application()
    app.add_routes(routes)
    app.on_shutdown.append(close_sockets)
    web.run_app(app, host='::1', port=0xabe5)

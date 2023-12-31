import asyncio
import aiohttp
from time import perf_counter


base_url = "https://mt1.google.com"
headers = {"User-Agent": "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/119.0"}
requests = [ "/vt/lyrs=s&x=12108&y=25490&z=16", "/vt/lyrs=s&x=1515&y=3197&z=13", "/vt/lyrs=s&x=12108&y=25491&z=16", "/vt/lyrs=s&x=1514&y=3198&z=13" ]

def urls(n_reqs: int):
    for _ in range(n_reqs):
        yield "https://python.org"

async def get(session: aiohttp.ClientSession, url: str):
    async with session.get(url) as response:
        _ = await response.read()
        print("read:", len(_))
                # response = self.conn.getresponse()
                # print(response.status, response.reason)
                # data = response.read()  # This will return entire content.

async def main(requests):
    session = aiohttp.ClientSession(base_url=base_url, headers=headers, timeout=aiohttp.ClientTimeout(total=10))
    #async with session:
    # async with aiohttp.ClientSession() as session:
    await asyncio.gather(
        *[get(session, request) for request in requests]
    )
        
    await asyncio.gather(
        *[get(session, request) for request in requests]
    )


if __name__ == "__main__":
    start = perf_counter()
    # asyncio.run(main(n_reqs))
    asyncio.run(main(requests))
    asyncio.run(main(requests))
    end = perf_counter()
    print(end-start)
    # print(f"{n_reqs / (end - start)} req/s")


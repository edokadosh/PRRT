import json
import random

w = 200
h = 200

freedom = 10

num_obstacles = (h*w)//freedom

obs = {}

class point:
    x = 0
    y = 0
    def __init__(self, _x, _y):
        self.x = _x
        self.y = _y

class obstacle:
    points = []
    def __init__(self, p1, p2, p3, p4):
        self.points = [p1, p2, p3, p4]

def add_obs(d, o : obstacle):
    newObs = obs.copy()
    newObs['poly'] = [[p.x, p.y] for p in o.points]
    d['obstacles'].append(newObs)

def random_point():
    return point(random.random()*w, random.random()*h)

def point_near_start_or_end(p : point):
    return (p.x < 4 and p.y < 4) or (p.x > w-8 and p.y > h-8) 

def main():
    d = None
    with open("empty_scene.json", 'r') as f:
        d = json.load(f)
    
    global obs
    obs = d['obstacles'][0]
    d['obstacles'] = []
    d['robots'][0]['start'] = [3,3]
    d['robots'][0]['end'] = [w-3,h-3]


    add_obs(d, obstacle(point(0,0), point(0,1), point(w,1), point(w,0)))
    add_obs(d, obstacle(point(0,0), point(1,0), point(1,h), point(0,h)))
    add_obs(d, obstacle(point(w,h), point(w,h-1), point(0,h-1), point(0,h)))
    add_obs(d, obstacle(point(w,h), point(w-1,h), point(w-1,0), point(w,0)))

    p1 = point(0,0)

    for i in range(num_obstacles//2):
        p1 = random_point()
        while point_near_start_or_end(p1):
            p1 = random_point()

        o = obstacle(p1, point(p1.x, p1.y+4), point(p1.x+1, p1.y+4), point(p1.x+1, p1.y))

        add_obs(d, o)

    for i in range(num_obstacles//2):
        p1 = random_point()
        while point_near_start_or_end(p1):
            p1 = random_point()

        o = obstacle(p1, point(p1.x, p1.y+1), point(p1.x+4, p1.y+1), point(p1.x+4, p1.y))
        

        add_obs(d, o)
    
    with open('generated_big_scene_1.5.json', 'w') as f:
        json.dump(d, f)
        


if __name__ == "__main__":
    main()



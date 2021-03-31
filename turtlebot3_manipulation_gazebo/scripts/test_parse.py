import os
import xml.etree.ElementTree as ET
import xacro
import networkx as nx
import matplotlib.pyplot as plt

# findings:
# probably don't need pose 0 0 0 0 0 0 0 at start of additional SDF deps
# caster joints probably need pose


def extract_attribs(e, g, edge_labels, links, joints):
    if e.tag == 'link':
        g.add_node(e.attrib['name'])
        print('Added link: {}'.format(e.attrib['name']))
        links += 1
    elif e.tag == 'joint':
        joints += 1
        parent = ''
        child = ''

        for c in e:
            if c.tag == 'child':
                child = c.attrib['link']
            elif c.tag == 'parent':
                parent = c.attrib['link']
        if child == '' or parent == '':
            print("WARNING: JOINT {} DEFINITION IS INCOMPLETE".format(e.attrib['name']))

        else:
            g.add_edge(child, parent)
            edge_labels[(child, parent)] = e.attrib['name']
    return g, edge_labels, links, joints


def main():
    fp = 'turtlebot3_waffle_manipulator_gazebo.urdf'
    tree = ET.parse(fp)
    root = tree.getroot()
    links = 0
    joints = 0

    g = nx.DiGraph()
    edge_labels = {}

    for e in root:
        print(e)
        if e.tag == 'gazebo':
            for ge in e:
                g, edge_labels, links, joints = extract_attribs(ge, g, edge_labels, links, joints)
        else:
            g, edge_labels, links, joints = extract_attribs(e, g, edge_labels, links, joints)

    print("{} Links; {} Joints".format(links, joints))
    pos = nx.spring_layout(g)
    nx.draw_networkx_edge_labels(g, pos, edge_labels=edge_labels)
    plt.axis('off')
    plt.show()


if __name__ == "__main__":
    main()

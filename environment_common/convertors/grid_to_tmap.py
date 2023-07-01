
def main(grid, yml_cats):
    dims = (yml_cats['grid']['resolution']['x'], yml_cats['grid']['resolution']['y'])
    save_as = yml_cats['CONFIG']
    renders = yml_cats['renderings']

    # Get standard vertice points
    verts = get_verts()

    # Get tmap base setup
    tmap_details = {'gen_time': '2022-06-23_13-10-11', 'location': 'auto_gen'}
    tmap = get_tmap(tmap_details)

    for ri,r in enumerate(grid):
        for ci,c in enumerate(grid[ri]):

            # Skip pole nodes
            cat = grid[ri][ci]
            ren = renders[cat]['sdf']
            if ren == 'pole': continue

            # Generate generic node template
            name = "%s-r%s-c%s"%(ren, ri, ci)
            x = ci*dims[0] - (0.5*dims[0]*(len(grid)-1))
            y = ri*dims[1] - (0.5*dims[1]*(len(grid[ri])-1))
            node_details = {'name':name,
                            'location':tmap_details['location'],
                            'vert': renders[cat]['vert'],
                            'x':x,
                            'y':y,
                            'restrictions':renders[cat]['restrictions'][cat]}
            node = get_node(node_details)

            #Get neighbouring edges
            neighbours = get_neighbours(grid, ri, ci)
            edge_list = []

            for nei in neighbours:

                #Skip if pole
                cat2 = nei[2]
                ren2 = renders[cat2]['sdf']
                if ren2 == 'pole': continue

                # Generate edge
                neighbour = str(grid[nei[0], nei[1]])
                name2 = "%s-r%s-c%s"%(ren2, nei[0], nei[1])
                edge_details = {'name':name,
                                'name2':name2,
                                'action':'move_base',
                                'action_type':'move_base_msgs/MoveBaseGoal',
                                'restrictions': renders[cat]['restrictions'][cat2]}
                edge_list += [get_edge(edge_details)]

            # Add edges or empty list
            if edge_list:
                node += """    edges:
""" + ''.join(edge_list)
            else:
                node += """    edges: []
"""

            tmap+=node
    verts+=tmap

    # Save TMap file
    tmap = verts
    if save_as:
        path = rospkg.RosPack().get_path("config_generator")+"/../scenarios/scenario___%s/config/tmaps/tmap.tmap2"%(save_as)
        with open(path, 'w+') as f:
            f.write(tmap)


if __name__ == '__main__':
    main()

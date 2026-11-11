"""Target scan overrides without changing independent PointCloud2 sources."""


def scan_topic_rewrites(config, topic):
    rewrites = {}
    for name in ('local_costmap', 'global_costmap'):
        costmap = config[name][name]['ros__parameters']
        for plugin in costmap.get('plugins', []):
            layer = costmap.get(plugin, {})
            for source in layer.get('observation_sources', '').split():
                if layer[source].get('data_type', 'LaserScan') == 'LaserScan':
                    key = f'{name}.{name}.ros__parameters.{plugin}.{source}.topic'
                    rewrites[key] = topic
    return rewrites

import json
import os

from maps import feature_dict


class LidarLineMapLayer(object):
    def __init__(self, map_dir):
        self.map_dir = map_dir

    def __getitem__(self, key):
        return self.get_batch(key)

    def get_batch(self, batch_id):
        if not self.batch_exists(batch_id):
            return None

        fn = self.get_batch_filename(batch_id)
        if not os.path.exists(fn):
            return None
        return feature_dict.load_from_file(fn)

    # --------------------------------------
    # Disk Operations
    # --------------------------------------

    def batch_exists(self, batch_id):
        fn = self.get_batch_filename(batch_id)
        return os.path.exists(fn) and os.path.isfile(fn)

    def get_batch_filename(self, tile_id):
        return os.path.join(self.map_dir, '{}.json'.format(tile_id))

    def get_feature(self, ref):
        batch = self.get_batch(ref['batch_id'])
        if batch is None:
            return None

        feature_type = ref['type'].replace('_ref', '')
        return batch.get_features(feature_type).get(ref)

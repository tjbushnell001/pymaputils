import glob
import os

from maps import feature_dict


class LidarLineLayer(object):
    def __init__(self, map_dir):
        self.map_dir = map_dir
        if not os.path.exists(self.map_dir):
            self.sub_dirs = []
        else:
            self.sub_dirs = next(os.walk(self.map_dir))[1]

    def __getitem__(self, key):
        return self.get_section(key)

    def get_section(self, section_id):
        if not self.section_exists(section_id):
            return None

        fn = self.get_section_path(section_id)
        if fn is None:
            return None
        return feature_dict.load_from_file(fn)

    def get_sections(self):
        sections = []
        for sub_dir in self.sub_dirs:
            for fn in glob.glob(os.path.join(self.map_dir, sub_dir, '*.json')):
                sections.append(os.path.splitext(os.path.basename(fn))[0])
        return sections

    # --------------------------------------
    # Disk Operations
    # --------------------------------------

    def section_exists(self, section_id):
        return self.get_section_path(section_id) is not None

    def get_section_path(self, section_id):
        for sub_dir in self.sub_dirs:
            fn = os.path.join(self.map_dir, sub_dir, "{}.json".format(section_id))
            if os.path.exists(fn) and os.path.isfile(fn):
                return fn
        return None

    def get_feature(self, ref):
        section = self.get_section(ref['section_id'])
        if section is None:
            return None

        feature_type = ref['type'].replace('_ref', '')
        return section.get_features(feature_type).get(ref)

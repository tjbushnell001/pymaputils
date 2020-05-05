node (label: 'aws-jenkins-slave-worker') {
  stage('Checking out code') {
    checkout scm: [
      $class: 'GitSCM',
      branches: scm.branches,
      doGenerateSubmoduleConfigurations: false,
      extensions: scm.extensions +
        [[$class: 'SubmoduleOption', parentCredentials: true]] +
        [$class: 'GitLFSPull'],
      userRemoteConfigs: scm.userRemoteConfigs
    ]

    sh "rm -rf tiled_maps"
    dir('tiled_maps') {
      git branch: 'master', url: 'git@github.com:embarktrucks/tiled_maps.git'
    }
  }

  stage('Running pylint') {
    docker.image('python:2').inside('-u root') {
      sh "pip install --no-cache-dir pylint"
      sh "./pylint"
    }
  }

  stage('Installing dependencies') {
    docker.image('python:2').inside('-u root') {
      sh "pip install -r requirements.txt"
    }
  }

  stage('Running unittests') {
    docker.image('python:2').inside('-u root') {
      sh "python -m unittest discover"
    }
  }

  stage('Running map linter') {
    docker.image('python:2').inside('-u root') {
      sh "python scripts/lint_map.py --map_dir tiled_maps/usa"
    }
  }
}

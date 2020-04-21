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
  }

  stage('Running pylint') {
    docker.image('python:2').inside('-u root') {
      sh "pip install --no-cache-dir pylint"
      sh "./pylint"
    }
  }

  stage('Running unittests') {
    docker.image('python:2').inside('-u root') {
      sh "pip install -r requirements.txt"
      sh "python -m unittest discover"
    }
  }
}

(window.webpackJsonp=window.webpackJsonp||[]).push([[21],{371:function(a,s,t){"use strict";t.r(s);var e=t(42),r=Object(e.a)({},(function(){var a=this,s=a.$createElement,t=a._self._c||s;return t("ContentSlotsDistributor",{attrs:{"slot-key":a.$parent.slotKey}},[t("h1",{attrs:{id:"notes"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#notes"}},[a._v("#")]),a._v(" Notes")]),a._v(" "),t("p",[a._v("Notes on development and debugging commands. For the authors reference mainly!")]),a._v(" "),t("h2",{attrs:{id:"update-files"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#update-files"}},[a._v("#")]),a._v(" Update files")]),a._v(" "),t("p",[a._v('Scripts are used to fetch latest updates and collect static files. If <branch name> is empty, defaults to "master".')]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token builtin class-name"}},[a._v("cd")]),a._v(" silvia\n$ ./update_branch "),t("span",{pre:!0,attrs:{class:"token operator"}},[a._v("<")]),a._v("branch name"),t("span",{pre:!0,attrs:{class:"token operator"}},[a._v(">")]),a._v("\n")])])]),t("h2",{attrs:{id:"django"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#django"}},[a._v("#")]),a._v(" Django")]),a._v(" "),t("p",[a._v("Run django dev server on pi")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ python manage.py runserver "),t("span",{pre:!0,attrs:{class:"token number"}},[a._v("192.168")]),a._v(".0.9:8000 \n")])])]),t("h2",{attrs:{id:"message-broker"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#message-broker"}},[a._v("#")]),a._v(" Message broker")]),a._v(" "),t("h3",{attrs:{id:"useful-redis-commands"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#useful-redis-commands"}},[a._v("#")]),a._v(" Useful Redis commands")]),a._v(" "),t("p",[a._v("Start Redis server")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" redis-server "),t("span",{pre:!0,attrs:{class:"token comment"}},[a._v("# Start!")]),a._v("\n")])])]),t("p",[a._v("Clear queues")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ redis-cli flushall\n")])])]),t("h3",{attrs:{id:"celery"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#celery"}},[a._v("#")]),a._v(" Celery")]),a._v(" "),t("p",[a._v("Start celery and celery-beat")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token builtin class-name"}},[a._v("cd")]),a._v(" ~/silvia/silvia\n$ celery -A silvia worker -l info\n$ celery -A silvia beat -l info --scheduler django_celery_beat.schedulers:DatabaseScheduler\n")])])]),t("p",[a._v("Inspect celery using flower")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token builtin class-name"}},[a._v("cd")]),a._v(" ~/silvia/silvia\n$ flower -A silvia --address"),t("span",{pre:!0,attrs:{class:"token operator"}},[a._v("=")]),t("span",{pre:!0,attrs:{class:"token number"}},[a._v("192.168")]),a._v(".0.6 --port"),t("span",{pre:!0,attrs:{class:"token operator"}},[a._v("=")]),t("span",{pre:!0,attrs:{class:"token number"}},[a._v("5555")]),a._v("\n")])])]),t("h2",{attrs:{id:"supervisor"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#supervisor"}},[a._v("#")]),a._v(" Supervisor")]),a._v(" "),t("p",[a._v("Start")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" supervisord -c /etc/supervisor/supervisord.conf\n")])])]),t("p",[a._v("See processes")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" supervisorctl\n")])])]),t("p",[a._v("Restart processes")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$  x celery_workers:silvia_worker1\n")])])]),t("p",[a._v("Stop/start all")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" supervisorctl stop all\n$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" supervisorctl start all\n")])])]),t("p",[a._v("Reload config")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" supervisorctl reread\n$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" supervisorctl update\n")])])]),t("p",[a._v("Inspect logs:")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("tail")]),a._v(" /var/log/celery/silvia_worker.log\n$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("tail")]),a._v(" /var/log/celery/silvia_beat.log\n")])])]),t("h2",{attrs:{id:"apache"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#apache"}},[a._v("#")]),a._v(" Apache")]),a._v(" "),t("p",[a._v("Apache can be restarted using:")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("service")]),a._v(" apache2 restart\n")])])]),t("p",[a._v("Or stopped:")]),a._v(" "),t("div",{staticClass:"language-ash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-text"}},[t("code",[a._v("$ sudo service apache2 stop\n")])])]),t("p",[a._v("Check log errors usuing:")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("nano")]),a._v(" /var/log/apache2/error.log\n")])])]),t("h2",{attrs:{id:"arduino-from-pi"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#arduino-from-pi"}},[a._v("#")]),a._v(" Arduino from Pi")]),a._v(" "),t("p",[a._v("See serial output. If not installed, install screen:")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("apt")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("install")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("screen")]),a._v("\n")])])]),t("p",[a._v("Then view serial output:")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("screen")]),a._v(" /dev/ttyACM0 "),t("span",{pre:!0,attrs:{class:"token number"}},[a._v("9600")]),a._v("\n")])])]),t("p",[a._v("To stop:\n'CTRL+a' and then 'k' then 'y'")]),a._v(" "),t("h2",{attrs:{id:"pi-errors"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#pi-errors"}},[a._v("#")]),a._v(" Pi Errors")]),a._v(" "),t("p",[a._v("I/O error fix (read only file system):")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("touch")]),a._v(" /boot/forcefsck\n$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("shutdown")]),a._v(" -r now\n")])])]),t("h2",{attrs:{id:"changing-database-to-postgresql"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#changing-database-to-postgresql"}},[a._v("#")]),a._v(" Changing Database to PostgreSQL")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("apt-get")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("install")]),a._v(" postgresql libpq-dev postgresql-client postgresql-client-common python-dev -y\n$ pip "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("install")]),a._v(" psycopg2\n$ "),t("span",{pre:!0,attrs:{class:"token builtin class-name"}},[a._v("cd")]),a._v(" ~/silvia/silvia\n$ python manage.py dumpdata --exclude"),t("span",{pre:!0,attrs:{class:"token operator"}},[a._v("=")]),a._v("contenttypes --exclude"),t("span",{pre:!0,attrs:{class:"token operator"}},[a._v("=")]),a._v("auth.Permission "),t("span",{pre:!0,attrs:{class:"token operator"}},[a._v(">")]),a._v(" datadump.json\n")])])]),t("p",[a._v("Test connection:")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("sudo")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("su")]),a._v(" - postgres\n$ psql\n$ CREATE DATABASE silviadatabase"),t("span",{pre:!0,attrs:{class:"token punctuation"}},[a._v(";")]),a._v("\n$ CREATE "),t("span",{pre:!0,attrs:{class:"token environment constant"}},[a._v("USER")]),a._v(" databaseadmin WITH PASSWORD "),t("span",{pre:!0,attrs:{class:"token string"}},[a._v("'databasepwd'")]),t("span",{pre:!0,attrs:{class:"token punctuation"}},[a._v(";")]),a._v("\n$ ALTER ROLE databaseadmin SET client_encoding TO "),t("span",{pre:!0,attrs:{class:"token string"}},[a._v("'utf8'")]),t("span",{pre:!0,attrs:{class:"token punctuation"}},[a._v(";")]),a._v("\n$ ALTER ROLE databaseadmin SET default_transaction_isolation TO "),t("span",{pre:!0,attrs:{class:"token string"}},[a._v("'read committed'")]),t("span",{pre:!0,attrs:{class:"token punctuation"}},[a._v(";")]),a._v("\n$ ALTER ROLE databaseadmin SET timezone TO "),t("span",{pre:!0,attrs:{class:"token string"}},[a._v("'GB'")]),t("span",{pre:!0,attrs:{class:"token punctuation"}},[a._v(";")]),a._v("\n$ GRANT ALL PRIVILEGES ON DATABASE silviadatabase TO databaseadmin"),t("span",{pre:!0,attrs:{class:"token punctuation"}},[a._v(";")]),a._v("\n$ "),t("span",{pre:!0,attrs:{class:"token punctuation"}},[a._v("\\")]),a._v("q\n$ "),t("span",{pre:!0,attrs:{class:"token builtin class-name"}},[a._v("exit")]),a._v("\n")])])]),t("p",[a._v("Change database settings in "),t("code",[a._v("settings.py")]),a._v(":")]),a._v(" "),t("div",{staticClass:"language- extra-class"},[t("pre",{pre:!0,attrs:{class:"language-text"}},[t("code",[a._v("DATABASES = {\n    'default': {\n        'ENGINE': 'django.db.backends.postgresql',\n        'NAME': 'silviadatabase',\n        'USER': 'databaseadmin',\n        'PASSWORD': 'databasepwd',\n        'HOST': '127.0.0.1',\n        'PORT': '',  # Default\n    }\n}\n")])])]),t("p",[a._v("Migrate and recreate superuser")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ python manage.py migrate\n$ python manage.py createsuperuser\n")])])]),t("h2",{attrs:{id:"gitignore-changes"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#gitignore-changes"}},[a._v("#")]),a._v(" .gitignore changes")]),a._v(" "),t("p",[a._v("Remove files from git based upon .gitignore changes")]),a._v(" "),t("div",{staticClass:"language-bash extra-class"},[t("pre",{pre:!0,attrs:{class:"language-bash"}},[t("code",[a._v("$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("git")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("rm")]),a._v(" -r --cached "),t("span",{pre:!0,attrs:{class:"token builtin class-name"}},[a._v(".")]),a._v("\n$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("git")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("add")]),a._v(" "),t("span",{pre:!0,attrs:{class:"token builtin class-name"}},[a._v(".")]),a._v("\n$ "),t("span",{pre:!0,attrs:{class:"token function"}},[a._v("git")]),a._v(" commit -m "),t("span",{pre:!0,attrs:{class:"token string"}},[a._v('"update files based upon .gitignore changes"')]),a._v("\n")])])]),t("h3",{attrs:{id:"scale"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#scale"}},[a._v("#")]),a._v(" Scale")]),a._v(" "),t("h3",{attrs:{id:"display-images"}},[t("a",{staticClass:"header-anchor",attrs:{href:"#display-images"}},[a._v("#")]),a._v(" Display Images")]),a._v(" "),t("p",[a._v("XBM images can be converted at: "),t("a",{attrs:{href:"https://www.online-utility.org/image/convert/to/XBM",target:"_blank",rel:"noopener noreferrer"}},[a._v("www.online-utility.org"),t("OutboundLink")],1)])])}),[],!1,null,null,null);s.default=r.exports}}]);
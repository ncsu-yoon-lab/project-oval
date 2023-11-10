To make your feature branch:
```shell
# first pull latest changes
git pull

# make sure you are on the development branch
git checkout development

# checkout to your branch
git checkout -b {first_name}_{description}

# push your new branch to the remote repo
git push -u origin [branch-name]
```

Then you can continue to work on your branch with these basic commands:
```shell
# to pull changes
git pull

# to push a change
git add {name of file to add}
# or to add all changed files
git add .

# then write a commit message
git commit -m "message"

# finally push to repo
git push
```

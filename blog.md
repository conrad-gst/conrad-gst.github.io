<!--

---
layout: default
title: Blog
---
# Latest Posts

<ul>
  {% for post in site.posts %}
    <li>
      <h3><a href="{{ post.url }}">{{ post.title }}</a></h3>
      <p>{{ post.date | date_to_string }}</p>
      <p>{{ post.excerpt }}</p>
      <a href="{{ post.url }}" style="color: blue; color: blue; text-decoration: none;"> ... continue reading</a>
    </li>
  {% endfor %}
</ul>

-->
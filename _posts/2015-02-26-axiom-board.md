---
title:      SYSC 2003 Axiom Board Features
permalink:  /SYSC2003/axiom-board
layout:     wrapper
heading:
folder:     sysc2003
contentswap:
- header: LEDs
  url:    leds
- header: LCD
  url:    lcds
- header: DC Motor
  url:    dc-motor
- header: Stepper
  url:    stepper
---

<nav>

  {% for item in page.contentswap %}

    <a href="javascript:content_swap('{{ item.url }}', 'axiom-board-code');">
      <div style="display:inline-block; text-align:center; line-height:40px; background:#333; width:24%; height:40px; margin-bottom:3px;">
          <strong>{{ item.header }}</strong>
      </div>
    </a>
  {% endfor %}

</nav>

<div id="axiom-board-code">

{% highlight c %}

{% include code_snippets/{{ page.folder }}/imports.c %}
{% include code_snippets/{{ page.folder }}/delay.c %}
{% include code_snippets/{{ page.folder }}/rolling_blink.c %}
{% include code_snippets/{{ page.folder }}/flash.c %}
{% include code_snippets/{{ page.folder }}/leds.c %}

{% endhighlight %}

</div>

<div id="leds" style="display:none">

{% highlight c %}

{% include code_snippets/{{ page.folder }}/imports.c %}
{% include code_snippets/{{ page.folder }}/delay.c %}
{% include code_snippets/{{ page.folder }}/rolling_blink.c %}
{% include code_snippets/{{ page.folder }}/flash.c %}
{% include code_snippets/{{ page.folder }}/leds.c %}

{% endhighlight %}

</div>

<div id="lcds" style="display:none">

{% highlight c %}

{% include code_snippets/{{ page.folder }}/imports.c %}
{% include code_snippets/{{ page.folder }}/delay.c %}
{% include code_snippets/{{ page.folder }}/LCD.c %}

{% endhighlight %}

</div>

<div id="stepper" style="display:none">

{% highlight c %}

{% include code_snippets/{{ page.folder }}/imports.c %}
{% include code_snippets/{{ page.folder }}/stepper_motor.c %}

{% endhighlight %}

</div>

<div id="dc-motor" style="display:none">

{% highlight ca65 %}

{% include code_snippets/{{ page.folder }}/dcmotor.s %}

{% endhighlight %}

</div>

<div id="output-compare" style="display:none">

{% highlight c %}

{% include code_snippets/{{ page.folder }}/imports.c %}
{% include code_snippets/{{ page.folder }}/output_compare.c %}

{% endhighlight %}

</div>
